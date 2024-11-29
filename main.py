#/usr/bin/python3
# -*- coding: utf-8 -*-
''' ------------------------------------------------------------------------------------------------
Regulation chauffage eau plancher chauffant
    * Acquisition thermometres  DS18x20 (Pin 22)
        Text : Température extérieure
        Tint : Température ambiante
        Tcuv : Température cuve solaire niveau interméiaire
        Tv3v : Température sortie vanne 3V
    * Lecture téléinformation compteur  (Pin 4)
        Gestion déléstage si chauffage electrique
        Cumul consommation electrique chauffage (Heures creuses et pleines)
    * Calcul température consigne eau
        Loi d'eau lineaire par segment(4) en fonction de Consigne T amb., T ext., T amb.
    * Regulation de la vanne 3 voies sortie circuit solaire
        Prise en compte température stock, Consigne T eau chauffage
        Regulation en impulsion + ( Pin 20) et - (Pin 21) de la vanne avec attente après chaque réglage
        Calcul position vanne (%) d'ouverture
    * Regulation chaudiere électrique pour complément solaire
        Regulation par PID et pilotage de 4 résistances de 1.6 kW
    * Commande circulateur
----------------------------------------------------------------------------------------------- '''
# Essai ajout ligne
import json
import sys
import time
#import gc
import _thread
import machine
import onewire
import pycom
from machine import RTC, UART, Pin, Timer, WDT
from network import WLAN
#from PID import PID
from umqtt import MQTTClient

# Constantes et configuration
#-------------------------- DEBUG ---------------------------
DEBUG = True
#DEBUG = False
#------------------ Emulation compteur EDF ------------------
#SIMU = const(0)
#SIMU=const(1) 
#------------------- Watchdog -------------------------------
#WATCH_DOG = False
WATCH_DOG = True
STX = b'\x02'
ETX = b'\x03'
ON = const(1)              # Pour activer sorties logiques
OFF = const(0)
NBTHERMO = const(4) # Nombre de thermometres OneWire
TYPE_CPT = 'LINKY' 
TYPE_CONTRAT = 'ZEN_WEEKEND_PLUS'  # Contrat EDF ZEN WEEKEND
#TYPE_CONTRAT = "HISTORIQUE"         # Contrat classique heure creuse
jours_hc ={2,5,6}                   # Jours heures creuses 24H
plage_hc = [2.5, 7.5, 13.5, 16.5]    # Heures creuses autres jours en H et cH

# Trame d'emission simulation connexion compteur (téléinfo edf)
# if SIMU == 1:
#    trame_edf = STX + \
#        b'\nADCO 123456789012 $\r' + \
#        b'\nOPTARIF HC.. $\r' + \
#        b'\nISOUSC 45 $\r' + \
#        b'\nHCHC 12345678 $\r' + \
#        b'\nHCHP 12345678 $\r' + \
#        b'\nPTEC HP.. $\r' + \
#        b'\nIINST 012 $\r' + \
#        b'\nADPS 123 $\r' + \
#        b'\nIMAX 123 $\r' + \
#        b'\nPAPP 12345 $\r' + \
#        b'\nHHPHC 1 $\r' + \
#        b'\nMOTDETAT 123456 $\r' + \
#        ETX

# ------------------ Constantes ------------------------------------
#
#----------- Parametres par defaut pour creation fichiers ----------------------------
# T eau fonction de T ext, Consigne ambiante et ecart consigne ambiante - T ambianteregul_chauffe.py
# Parametres pour calcul loi d'eau lineaire par segment : 
# (offset(°C), (t_max_zon1(°C), pente1), (t_max2(°C),_zon2 pente2), ...,Use_Temp_int True/False)
param_cons = [20.0, (10, 0.47), (20, 0.45), (30, 0.42), (40, 0.39), True]

# Parametres pour regulation vanne
# (T cuve mini utilisable(°C), Bandemorte regul(°C), t(s) pulse+/-, t(s) attente, t(s) ouverture 0-100%)
param_vanne = [25.0, 0.5, 3, 60, 120]

# Parametres gestion commande electrique thermoplongeur
# (DT calcul conso en HC, DT calcul conso en HP, Resistance unitaire (Ohms), tension(V) unitaire par résistance)
param_thermop = [25.0, 6.0, 26.0, 225]

# Parametres de fonctionnement
# (Consigne T amb,(°C), Marche=2 : Circulateur en continu, Marche = 1 : Circulateur controlé par temp ambiante,
# Marche = 0 : Arret chauffage)
param_fonct = [19.5, 0]

# -------------------------  Definitions ports entrées et sorties
p_circu = 'P19'                 # Cde circulateur
p_v3v_p = 'P20'                 # Cde + vanne 3 voies
p_v3v_m = 'P21'                 # Cde - vanne 3 voies
p_bus_ow = 'P22'                # bus OneWire pour DS18X20
p_R1 = 'P5'                     # Cde resistance R1
p_R2 = 'P6'                     # Cde resistance R2
p_R3 = 'P7'                     # Cde resistance R3
p_libre = 'P8'                  # Libre 
p_hc = 'P9'                     # Heures creuses

T_NOM_TH =['Text', 'Tint', 'Tcuv', 'Tv3v'] # Si changement DS18 modifié fichier thermo.at
T_CYCLE_S = 1800            # Periode PWM circulateur en sec.
SCALE_PWM = 4.0             # (T salon - T consigne) pour calcul PWM circulateur

# WIFI connexion data, ID , PWD
WIFI_C = ('192.168.0.52', '255.255.255.0', '192.168.0.254', '212.27.40.240')
SSID='freebox_PC'
PWID='parapente'

# NTP server 
#NTP_SERVER = "ntp.deuza.net"
NTP_SERVER = "pool.ntp.org"

# Broker MQTT Mosquitto sur Raspberry PI 4 sur reseau local IP forcé par configuration routeur freebox
MQTT_server = "192.168.0.41"         
MQTT_PORT = 1883
MQTT_USER =''
MQTT_PASSW = ''
# Broker cloudmqtt.com version payante depuis 30/04/2020 (5$/mois)
#MQTT_server = 'm23.cloudmqtt.com'  
#MQTT_PORT = 15201
#MQTT_USER = 'ixkefaip'
#MQTT_PASSW = 'Hf-lHiOHlb_p'

# Thread reception téléinformation compteur EDF
def edf_recv(serial):
    ''' Docstring de la fonction '''
    global dic_edf, new_lec
    encours = False
    while True:
        try:
            while serial.any() > 0:
                car=serial.read(1)
        #       print(car)
                if car==STX:
    #                print('STX')
                    mes = car
                    encours=True
                    continue
                if car!=ETX and encours is True:
                    mes+= car
                    continue
                elif encours is True and car == ETX:
    #                print('ETX')
                    mes+= car
                    encours=False
    ###                serial.readall()
                    tabl=[item.split(' ') for item in mes.decode().strip('\n\r\x02\x03').split('\r\n')]
                    dic=dict([[item[0], item[1]] for item in tabl])
                    lock.acquire()
                    dic_edf=dic.copy()
                    new_lec=True
                    lock.release()
                    machine.idle()
        except Exception as e :
            txtlog = 'Erreur thread EDF: ' + str(time.localtime()) + ' : ' +  e  + '\n'
            f=open('log.txt','a+') 
            f.write(txtlog)
            f.close()
            #print("Erreur thread lec EDF: ", dict, tabl)
            #machine.reset()

#
# Calcul consigne température chauffage (loi d'eau lineaire par segment)
#
def calc_cons_eau(SetP_amb, T_amb, T_ext, params):
    ''' Calcul consigne T eau (loi d'eau linéaire par segment) '''
    # Test si configuration utilisation ou non temperature interieur
    if params[5] == True :
        ecart_t = SetP_amb - T_ext + (SetP_amb - T_amb)   
    else : 
        ecart_t = SetP_amb - T_ext
    cons=params[0]
    v=0
    for i in range(len(params)-1):
        if ecart_t >= params[i+1][0]:
            v = params[i+1][0] - v
            cons += v * params[i+1][1]
            v=params[i+1][0]
        else:
            v = ecart_t - v
            cons+= v * params[i+1][1]
            break
    return cons
#
# Controle  circulateur et regulation
#
def cnt_circulateur(cons_amb,  t_amb, pin_cde, marche):
    ''' Commande circulateur '''
    
    global time_pwm_start_on, step_cnt, pulse_time 

    if marche == 1 :    # Marche chauffage
        if t_amb < (cons_amb +1.6) and t_amb != 0.0 :
            pin_cde(ON)
        elif t_amb > (cons_amb + 1.5) :
            pin_cde(OFF)
          
    elif marche == 2 :      # Marche chauffage avec circulateur actif en PWM sur cycle de T_CYCLE_S sec.
        if step_cnt == 0:
            r_activ =  1 - 1 / SCALE_PWM * (t_amb - cons_amb)  # Calcul durée pulse PWM circulateur
            time_pwm_start_on = time.time()
            if r_activ >= 1.0 :
                pulse_time = T_CYCLE_S          # Pulse = Periode
                step_cnt = 1
            elif r_activ <= 0 :
                pulse_time = 0                  # Pulse = 0
                step_cnt = 2
            else :
                step_cnt = 1
                pulse_time = T_CYCLE_S * r_activ  # Temps de pulse ON en sec 

        elif step_cnt == 1:
            pin_cde(ON)         # Cirulateur ON  
            if time.time() - time_pwm_start_on >= pulse_time and pulse_time < T_CYCLE_S:
                step_cnt = 2
            elif time.time() - time_pwm_start_on >= pulse_time :
                step_cnt = 0

        elif step_cnt == 2:
            pin_cde(OFF)         # Cirulateur OFF
            if (time.time() - time_pwm_start_on) >= T_CYCLE_S : 
                step_cnt = 0
    else:    
        pin_cde(OFF)
    return int(pin_cde.value())

# Regulation vanne 3 voies sur circuit solaire chauffage
#
class regul_vanne(object):
    ''' Agit par 2 commandes impulsion + et - sur l'ouverture et fermeture vanne,
    puis attent un temps mort avant de reagir a nouveau  '''

    def __init__(self, pin_p, pin_m, params):
        self.pin_p=pin_p
        self.pin_p.init(mode=pin_p.OUT)
        self.pin_m=pin_m
        self.pin_m.init(mode =pin_m.OUT)
        self.t_cuve_min=params[0]
        self.deadband = params[1]
        self.t_pulse = params[2] * 1000
        self.t_wait = params[3] * 1000
        self.t_move = params[4] * 1000
        self.etape = 0
        self.position = 0.0
        self.tempo = 0
        self.first_pos = False
        self.t_van_tm1 = 0.0
        self.dt_van = 0

    def run(self, t_cons_eau, t_cuve, t_sortie_vanne, t_cycl, circulateur, chauffage_on):
        ''' Active la régulation de la vanne '''
        if DEBUG :
            print('Etape cde vanne : ',  self.etape,  'Tempo',  self.tempo)
            print("DeltaT vanne: ", self.dt_van)
        else :
            print('Etape regulation : ', self.etape)
        
        #Gestion par Grafcet
        if self.etape == 0:         # Fermeture vanne position initiale de depart
            self.tempo = self.t_move
            self.pin_m(ON)             # active actionneur
            self.position = 0.0     # % ouverture vanne
            self.etape = 3
            self.t_van_tm1 = t_sortie_vanne  # _van_tm1 pour calcul pseudo dérivé de T vanne regul
            self.first_pos = True

        # Compense l'offset mecanique de la vanne pour optimisation temps

        if self.etape == 1 and circulateur == 1:
            self.tempo = 3000   # 12000 = 12s = 10 % d'offset
            self.pin_p(ON)          # active actionneur
            self.position = 2.5     # % ouverture vanne
            self.t_van_tm1 = t_sortie_vanne
            self.etape = 3
        
        # Regulation position vanne
        if self.etape == 2:         # Controle regulation vanne
            if circulateur == 1:     # Chauffage On
                self.t_van_tm1 = t_sortie_vanne
                # Ouvre + la vanne si T sortie vanne < consigne hors deadbande et dt <=  0
                if t_sortie_vanne < (t_cons_eau - self.deadband) and (self.dt_van <= 0):
                    self.tempo = self.t_pulse
                    self.pin_p(ON)
                    print('Pulse +')
                    if self.position < 100:
                        self.position += (self.t_pulse / self.t_move) * 100
                    else:
                        self.position = 100.0
                    self.etape = 3
                # Ouvre - la vanne si T sortie vanne > consigne hors deadbande et dt <=  0        
                elif t_sortie_vanne > (t_cons_eau + self.deadband) and (self.dt_van >= 0) :
                    self.tempo = self.t_pulse
                    self.pin_m(ON)
                    print('Pulse -')
                    if self.position > 0:
                        self.position -= (self.t_pulse / self.t_move ) * 100
                    else:
                        self.position = 0.0
                    self.etape = 3
                else:
                    self.tempo = self.t_wait
                    self.etape = 4
            else:
                self.etape = 0
        
        # Gestion durée pulse +/-
        if self.etape == 3:         # Gestion durée pulse +/- de correction
            if self.tempo > 0 :
                self.tempo -= t_cycl
            else:
                self.pin_p(OFF)
                self.pin_m(OFF)
                self.tempo = self.t_wait
                self.etape = 4
        
        # Gestion temps attente après modification ou non position vanne
        if self.etape == 4:        
            if self.tempo > 0 :
                self.tempo -= t_cycl
            else:
                self.pin_p(OFF)
                self.pin_m(OFF)
                if self.first_pos == True :
                    self.etape = 1
                    self.first_pos = False
                elif circulateur == 1:
                    self.etape = 2              # si circulateur off reste en étape 4
            self.dt_van = t_sortie_vanne - self.t_van_tm1 # Delta t durée attente > 0 la temperature monte
            if chauffage_on == 0:
                self.etape = 0
 
    def get_pos_vanne(self):
        ''' Get position vanne regulation '''
        return self.position

#
# Gestion commande thermoplongeur, delestage electrique, calcul puissance de chauffage en heures creuses et en heures pleines
#
class  ges_thermoplongeur(object):
    ''' Controle thermoplongeur '''
    def __init__(self, pin_r1,  pin_r2,  pin_r3,  pin_hc):
        self.pin_R = [Pin(pin_r1), Pin(pin_r2), Pin(pin_r3)]
        self.pin_R[0].init(mode=Pin.OUT)
        self.pin_R[1].init(mode=Pin.OUT)
        self.pin_R[2].init(mode=Pin.OUT)
        self.pin_hc = Pin(pin_hc)
        self.pin_hc.init(mode = Pin.OUT)
        self.nbr_activ = 0
# Recupere compteur dans NVRAM si existe, sinon les creent
        self.kw_hc = 0.0
        self.kw_hp = 0.0
        try:
            self.kw_hc = float(pycom.nvs_get('cpt_hc'))
        except:
            pycom.nvs_set('cpt_hc',0)
        try:
            self.kw_hp = float(pycom.nvs_get('cpt_hp'))
        except:
            pycom.nvs_set('cpt_hp',0)    
        self.puissance = 0.0
        self.t_encours = 'HP..'

# Fonction gestion pilotage résistance thermoplongeur et delestage
    def _delestage(self,nbR, nbr_activ, Idispo, Rmoy):
        self.nbr_activ = nbr_activ
        if nbR > 0 :                                # nbR ; nombre de résistances a activer au demarrage chauffage (1 à 3)
            if Idispo > (230/Rmoy) :                # I theorique par resistance
                if self.nbr_activ < 3:            # if self.nbr_activ < Nombre de résistances:
                    self.pin_R[self.nbr_activ].value(ON)  
                    self.nbr_activ += 1
                    #if DEBUG : print('Active une resistance', Idispo, self.nbr_activ)
            elif Idispo < 0 :                          # Si depassement courant souscrit
                # Delestage
                if self.nbr_activ > 0:
                    self.nbr_activ -= 1
                    self.pin_R[self.nbr_activ].value(OFF) 
                    #if DEBUG : print('Desactive une resistance', Idispo, self.nbr_activ)
            return self.nbr_activ
        else:                                           # On ne chauffe pas
            self.pin_R[0].value(OFF)    
            self.pin_R[1].value(OFF)
            self.pin_R[2].value(OFF)
            self.nbr_activ = 0
            return self.nbr_activ

# Gestion heures pleines heures creuses avec compteur electroniques ou\
# compteur Linky protocole historique, contrat historique ou contrat ZEN PLUS
    def _ges_hchp(self,type_cpt, type_contrat, data_edf):
        if type_cpt != 'LINKY' :    # ancien compteur electronique
            try:
                return data_edf['PTEC'] 
            except:
                return 'HP..'
        elif type_contrat == 'HISTORIQUE':     # Compteur Linky
            try:
                return data_edf['PTEC'] 
            except:
                return 'HP..'
        # Contrat non historique 'PTEC' pas utilisable, gestion par calendrier et time 
        elif type_contrat == 'ZEN_WEEKEND_PLUS':
            # Heure creuses tout le weekend, le mercredi, et les autres jours 2h30-7h30 13h30 16h30
            date_heure = time.localtime()
            heure_centi = date_heure[3] + (date_heure[4] / 60) # conversion en heure centiheure
            if date_heure[6] in jours_hc:
                return 'HC..'   # Jour HC sur 24h
            elif heure_centi >= plage_hc[0] and heure_centi < plage_hc[1]:
                return 'HC..'
            elif heure_centi >= plage_hc[2] and heure_centi < plage_hc[3]:
                return 'HC..'
            else :
                return 'HP..' 
        else:
            return 'HP..'

    def run(self, marche, t_cons_eau, t_cuve, data_edf, t_cycl,  params):
        ''' Gestion thermo plongeur'''
# Calcul du courant disponible pour le chauffage (delestage)
        try:
            self.iinst= int(data_edf['IINST'])
            ##self.imax= int(data_edf['ISOUSC'])
            self.imax = 32                      # ----------------- Pour test délesteur en avec abonnement 6kW -------------
            self.t_encours = self._ges_hchp(TYPE_CPT, TYPE_CONTRAT, data_edf)
            if DEBUG: print("Type tarif : ", self.t_encours)
            self.error = 0
        except :
            self.error = 1
            self.iinst =0
            self.imax = 25
            self.t_encours = 'HP..'
        self.Idispo = self.imax - self.iinst
# Positionne la sortie HC/HP
        if self.t_encours == 'HC..':
            self.pin_hc.value(ON)
        else:
            self.pin_hc.value(OFF)
   
# Controle chauffage par thermoplongeur
        # Marche chauffage et heures creuses
        if marche >= 1 and self.t_encours == 'HC..':
            # Chauffe si T cuve < Cons T eau + T acc:umulation HC
            if t_cuve + 0.2 < (params[0] + t_cons_eau) and t_cuve != 0.0 :
                # Cde 3 resistances thermo suivant delestage ou non (0 a 6 kW)
                self.nbr_toactiv = 1        # Démarre avec une resistance
            elif t_cuve > (params[0] + t_cons_eau) and t_cuve != 0.0 :
                self.nbr_toactiv = 0   
                # self.nbr_activ = 0
        # Marche et heures pleines
        else :
            if marche >= 1:
                # Chauffe si T cuve < Cons T eau + T accumulation HC
                #if t_cuve < params[0] + t_cons_eau : #----------- TEST -----------
                if t_cuve + 0.2 < (params[1] + t_cons_eau) and t_cuve != 0 :
                    print('Consigne chauffe cuve', t_cons_eau+params[1])
                    # Cde resistances thermo suivant delestage ou non (0 a 4 kW)
                    self.nbr_toactiv = 1        # Démarre avec 1 résistance
                elif t_cuve > (params[1] + t_cons_eau) and t_cuve != 0 :
                    # Reset cde resistance thermoplongeurs   
                    self.nbr_toactiv = 0   
                    # self.nbr_activ = 0
            else:
                # Resistances OFF
                self.nbr_toactiv = 0
                # self.nbr_activ = 0
        # Appel fonction pilotage sortie commande et gestion delestage 
        if DEBUG : 
            print("Thermoplongeur(actif,Nb R active,courant dispo): ",self.nbr_toactiv, self.nbr_activ, self.Idispo) 
        self.nbr_activ = self._delestage(self.nbr_toactiv, self.nbr_activ,self.Idispo, param_thermop[2])
        # Gestion comptage puissance chauffage
        self.current_theori = self.nbr_activ * (params[3] / params[2])  
        if self.iinst > self.current_theori - 2:  # Verifie que les resistances sont effectivement commandés
            self.puissance = self.current_theori * params[3] # Puissance en Watts
        else :
            self.puissance = 0
    
        if self.puissance > 0 :
            if self.t_encours == 'HC..' :
                self.kw_hc += self.puissance * t_cycl / 3600000  # conversions en w/h
                pycom.nvs_set('cpt_hc',int(self.kw_hc))     
            else :
                self.kw_hp += self.puissance * t_cycl / 3600000  # conversions en w/h
                pycom.nvs_set('cpt_hp',int(self.kw_hp))                       

    def get_power(self):
        ''' Docstring here '''
        return self.puissance
    
    def daily_save(self, year, nday):
        ''' Reset des compteurs HC et HP '''
        self.day_counter = str(year) + ',' + str(nday) \
        + ',' + str(self.kw_hc / 1000) \
        + ',' + str(self.kw_hp / 1000) + '\n'
        self.f= open('heatPower.csv', 'a+') 
        self.f.write(self.day_counter)
        self.f.close()
        pycom.nvs_set('cpt_hc', 0) 
        pycom.nvs_set('cpt_hp', 0)    
        self.kw_hc = 0
        self.kw_hp = 0    

    def get_energie(self):
        ''' Docstring here '''
        return self.kw_hc,  self.kw_hp

# Callbacks connexion MQTT protocole to free broker
#

def incoming_mess(topic, msg):
    ''' Docstring here '''
    global mes_send, cpt,  param_cons,  param_vanne,  param_chaudiere,  param_electrique, param_fonct
    if topic != None :
        if DEBUG:
            print("Subscribed : ", topic.decode(), msg.decode())
        if topic==b'/regchauf/send' and  msg == b'start':
            mes_send =True
            return
        if topic== b'/regchauf/send' and msg == b'stop':
            mes_send = False
            return
        if topic==b'/regchauf/cde' and msg == b'1': 
            param_fonct[1] = 1              # Regule avec T ext et T amb.
            f=open('p_fonct.dat', 'w')
            f.write(json.dumps(param_fonct))
            f.close()        
        if topic==b'/regchauf/cde' and msg == b'2':
            param_fonct[1] = 2              # Régule avec T ext seul (Circulateur en PWM)       
            f=open('p_fonct.dat', 'w')
            f.write(json.dumps(param_fonct))
            f.close()
            return
        if topic==b'/regchauf/cde' and msg == b'0':
            param_fonct[1] = 0
            f=open('p_fonct.dat', 'w')
            f.write(json.dumps(param_fonct))
            f.close()
            return
        if topic==b'/regchauf/cons':
            param_fonct[0] = float(msg.decode())
            f=open('p_fonct.dat', 'w')
            f.write(json.dumps(param_fonct))
            f.close()
            return


def lecture_fichiers():
    ''' Lecture fichiers configurations '''
    global param_fonct,  param_cons,  param_vanne,  param_thermop,  thermometres
    try:
        f=open('p_cons.dat', 'r')
        param_cons=json.loads(f.read())
        print('Lecture fichier p_cons.dat: ', param_cons)
    except:
        print('Erreur lecture fichier parametres calcul consigne eau')
        #Cree fichier parametres par defaut
        f=open('p_cons.dat','w')
        f.write(json.dumps(param_cons))
    finally:
        f.close()

    try:
        f=open('p_vanne.dat', 'r')
        param_vanne=json.loads(f.read())
        print('Lecture fichier p_vanne.dat: ',param_vanne)
    except:
        print('Erreur lecture fichier parametres vanne')
        #Cree fichier parametres par defaut
        f=open('p_vanne.dat','w')
        f.write(json.dumps(param_vanne))
    finally:
        f.close()

    try:
        f=open('p_thermop.dat', 'r')
        param_thermop=json.loads(f.read())
        print('Lecture fichier p_thermop.dat: ', param_thermop)
    except:
        print('Erreur lecture fichier parametres thermoplongeur')
        #Cree fichier parametres par defaut
        f=open('p_thermop.dat','w')
        f.write(json.dumps(param_thermop))
    finally:
        f.close()
    
    try:
        f=open('p_fonct.dat', 'r')
        param_fonct=json.loads(f.read())
        print('Lecture fichier p_fonct.dat: ', param_fonct)
    except:
        print('Erreur lecture fichier parametres fonctionnement')
        #Cree fichier parametres
        f=open('p_fonct.dat', 'w')
        f.write(json.dumps(param_fonct))        
    finally:
        f.close()
    
    # Lecture fichiers affections thermometres
    try:
        f=open('thermo.dat', 'r')
        data=f.read()
        #print(data)
        thermometres = json.loads(data)
        print('Lecture fichier thermometres thermo.dat: ', thermometres)
    except:
        print('Erreur lecture fichier thermo.dat')
        thermometres = {}
        dev = ds.roms
        if len(dev) == NBTHERMO:
    # Affectation des thermometres et enregistrement (converti ID en int: bug bytearray en json)
            for i,idn in enumerate(dev):
                thermometres[T_NOM_TH[i]] = int.from_bytes(idn,'little') 
            f=open('thermo.dat','w')
            print(thermometres)
            f.write(json.dumps(thermometres))
        else:
            print('Seulement ', len(dev), 'thermometres detectés sur ', NBTHERMO )
            time.sleep(5.0)
            machine.reset()  
    finally:
        f.close()

#
# ------------------------------ Main init -------------------------------------------
#
ds=onewire.DS18X20(onewire.OneWire(Pin(p_bus_ow)))
ser=UART(1)         # Init Uart 1 pour liaison compteur electrique EDF
ser.init(1200, bits=7,  parity=ser.EVEN, stop=1)
lock=_thread.allocate_lock()
task_recv =_thread.start_new_thread(edf_recv, (ser,))
rtc=RTC()
new_lec=False
mes=b''
temp={}
dev = ds.roms       # identification des capteurs DS18X20 raccordés
dic_edf={}
i=0
all_th = False
t_cycle= 0
# Sortie Cde circulateur
s_circul = Pin(p_circu, mode = Pin.OUT)
# Instances  regulation vanne, thermoplongeur
reg_v = regul_vanne(Pin(p_v3v_p),  Pin(p_v3v_m),  param_vanne)
reg_c = ges_thermoplongeur(p_R1, p_R2, p_R3, p_hc)
# Lecture fichiers parametres
lecture_fichiers()
etape_wifi = 0
mes_send = False
cpt_send=0
data_cpt={}
data_reel={}
alive = False
temp_tm1={}

pycom.heartbeat(False)
all_t_read = 0
jour_tm1 = 0
time_pwm_start_on = 0
step_cnt = 0
pulse_time = 0


# Init watchdog
if WATCH_DOG :
    wdog = WDT(timeout=25000)
on_time = False

for key in thermometres :
    temp_tm1[key] = -10000.0    # Pour temperature impossible (i nit)
flag = False
cpt_err_edf = 0
#
#---------------------------------- Main loop --------------------------------------------
#
while True:
#Lecture thermometres OneWire (Raffraichi un thermometre par boucle) avec filtrage d'une valeur incohérente
    for key in thermometres:
        start_t = time.ticks_ms() 
        pycom.rgbled(0x000800)                  # Rouge faible luminosité   
        idt= thermometres[key].to_bytes(8,'little')
        ds.start_convertion(idt)
        time.sleep(0.8)
        t_lue = ds.read_temp_async(idt)
        if t_lue == None :
            t_lue = temp[key]   # Recupére derniere valeur lue correcte
        #print (t_lue)
        if t_lue >=4095 :                   # ds18 debranché valeur = 4095.xx
            print('Defaut capteur ou non enregistre',key )
            temp[key]=0.0
            pycom.rgbled(0xff0000)
        else:
            temp[key] = t_lue
        if all_t_read == NBTHERMO:
            all_th = True
        else:
            all_t_read += 1
        if all_th :
            if DEBUG :    print('Temperatures : ',  temp)
# Recupere données compteur EDF
            lock.acquire()
            if new_lec:
                data_cpt=dic_edf.copy()
                cpt_err_edf = 0
            else:
                pycom.rgbled(0x0000ff)
                cpt_err_edf += 1
                if cpt_err_edf > 5 :
                    print('Defaut lecture teleinfo EDF')
                    machine.reset() 
            new_lec=False
            lock.release()
            # Modif suite Linky et Contrat non historique pour protocole MQTT 
            if TYPE_CPT == "LINKY" and TYPE_CONTRAT == "ZEN_WEEKEND_PLUS":
                data_cpt["PTEC"] = reg_c.t_encours
            if DEBUG :        print('Compteur EDF : ',  data_cpt)

    
    # Calcul consigne  temp eau chauffage
            cons_eau = calc_cons_eau(param_fonct[0], temp['Tint'],  temp['Text'], param_cons )
            if DEBUG :  print('Temp. consigne eau : ',  cons_eau)
    
    # Controle circulateur
            etat_circ = cnt_circulateur(param_fonct[0],  temp['Tint'], s_circul, param_fonct[1] )
            if DEBUG : print('Cde circulateur : ', etat_circ)
    
    # Regulation vanne 3 voie sortie reservoir tampon
            reg_v.run(cons_eau,  temp['Tcuv'],  temp['Tv3v'],  t_cycle,  etat_circ, param_fonct[1])
            position=reg_v.get_pos_vanne()
            if DEBUG : print('Ouverture vanne : ',  position,  ' %')
    
    # Controle commande thermoplongeur
            reg_c.run(param_fonct[1], cons_eau, temp['Tcuv'], data_cpt, t_cycle, param_thermop)
           
    # Puissance chauffage(W),  energie cumulé en HC et HP (kWh)
            if DEBUG :
                print('Puissance chauffe : ', reg_c.get_power(), ' W')
                print('Energie heures creuses : ',  reg_c.get_energie() [0] / 1000,  ' kWh')
                print('Energie heures pleines : ', reg_c.get_energie() [1] / 1000.0,  'kWh')

# Gestion protocole Telnet, FTP, MQTT en WiFI   print (wifi, mqtt_ok)
            if etape_wifi == 0:
                lswifi=[]
                wlan=WLAN(mode=WLAN.STA,antenna=WLAN.EXT_ANT)
                try:
                    lswifi=wlan.scan()
                except:
                    print('Pas de reseau WIFI')
                for r in lswifi:
# freebox et signal > -80 dB
                    if DEBUG: print(r)
                    if r[0] == SSID and r[4] > -87 :
                        wlan.ifconfig(config = ('192.168.0.52', '255.255.255.0', '192.168.0.254', '212.27.40.240'))
#                        wlan.ifconfig(config='dhcp')
                        wlan.connect(SSID, auth=(WLAN.WPA2, PWID), timeout=50)
                        #print('Connexion ?')
                        time.sleep(2)       # Indispensable
                        etape_wifi = 1
 
# Creation et initialisation protocole MQTT 
            if etape_wifi == 1:
                if wlan.isconnected(): 
                    print('Connecte WIFI : ',  wlan.ifconfig())

                    etape_wifi = 2
                else:
                    print('Wifi not connected')
            
            if etape_wifi == 2:        
                    try:
                        client =MQTTClient("chauffage",MQTT_server, MQTT_PORT, MQTT_USER, MQTT_PASSW)
                        client.set_callback(incoming_mess)
                        client.connect(clean_session=True)
                        client.subscribe('/regchauf/cde', qos= 0)
                        client.subscribe('/regchauf/send', qos= 0)
                        client.subscribe('/regchauf/cons', qos= 0)
                        print('Connected au serveur MQTT : ',  MQTT_server)
                        etape_wifi = 3
                    except:
                        #client.disconnect()
                        print ('Erreur connexion au seveur MQTT', MQTT_server)
                        time.sleep(1)
                        Wifi_etape = 1
                
# WIFI et MQTT Ok 
            if etape_wifi == 3:
                if not wlan.isconnected():  
                    etape_wifi = 0                    
                try:
                    client.check_msg()
                except:
                    client.disconnect()
                    print('MQTT check message entrant erreur')
                    etape_wifi = 1
                if mes_send is True:
                    try:
# Genere dictionnaire des données temps reel
                        data_reel['TEMP'] = temp
                        data_reel['EDF']= data_cpt
                        data_reel['CONS'] = cons_eau
                        data_reel['CIRC'] = etat_circ
                        data_reel['VANN'] = position
                        p = reg_c.get_power()
                        phc = reg_c.get_energie()[0]/1000
                        php = reg_c.get_energie()[1]/1000
                        data_reel['ELEC'] = {'PW': p, 'CHC' : phc, 'CHP' : php}
                        data_reel['FNCT'] = param_fonct
                        client.publish('/regchauf/mesur',json.dumps(data_reel))
                        if DEBUG : print('Publication mesures : ',  json.dumps(data_reel))
                    except:
                        if DEBUG : print('Publication mesures en erreur: ',  json.dumps(data_reel))
                        client.disconnect()
                        etape_wifi = 1
                alive = not alive
                client.publish('/regchauf/alive', bytes(str(alive), "utf8"))
# Test si reset sur watchdog 
                if machine.reset_cause() == machine.WDT_RESET and not on_time:
                    print('Reset par watchdog\n\n')
                    txtlog = 'Reset watchdog: ' + str(time.localtime()) + '\n'
                    f=open('log.txt','a+')
                    f.write(txtlog)   
                    f.close()
                    on_time = True
            if DEBUG: print('Etape Wifi: ', etape_wifi) 

#--------- Pour simulation liaison compteur Edf (jumper RX-TX loop)
# if SIMU == 1:  ser.write(trame_edf)
#--------------------------------------------------------------

# Initialise RTC avec service NTP
            gmt_time = time.gmtime()
            if gmt_time[2] != jour_tm1 :
                #rtc.ntp_sync("ntp.midway.ovh")
                rtc.ntp_sync(NTP_SERVER)

# Gestionchangement heure été/hiver
                jour_tm1 = gmt_time[2]
                time_hiver_ete = time.mktime((gmt_time[0], 3,31,1,30,0,0,0))   # 31/03 1h30 GMT
                dt_h_e = time.gmtime(time_hiver_ete)
                l_dt_h_e = list(dt_h_e)
                l_dt_h_e[2] -= dt_h_e[6] + 1              # Jour du dernier dimanche du mois
                dt_h_e = tuple(l_dt_h_e)
                time_hiver_ete = time.mktime(dt_h_e)
                time_ete_hiver = time.mktime((gmt_time[0], 10,31,0,30,0,0,0))   # 31/10 0h30 GMT
                dt_e_h = time.gmtime(time_ete_hiver)
                l_dt_e_h = list(dt_e_h) 
                l_dt_e_h[2] -= dt_e_h[6] + 1              # Jour du dernier dimanche du mois
                dt_e_h = tuple(l_dt_e_h)
                time_ete_hiver = time.mktime(dt_e_h)
                if time.mktime(gmt_time) >= time.mktime(dt_h_e) and time.mktime(gmt_time) <= time.mktime(dt_e_h):
                    time.timezone(7200)                # Local time = GMT + 2
                else:
                    time.timezone(3600)                # Local time = GMT + 1
                if DEBUG :print("-------- Heure locale -------  : ",time.localtime())

# Sauvegarde compteurs conso chauffage / 24h
        current_time = time.localtime() # Heure locale
        # print ("Heure locale : ",current_time)
# RTC initialiser par le reseau et heure = 23 minute = 59 ?
        if current_time[0] != 1970 and current_time[3] == 23 and current_time[4] == 59 :
            if param_fonct[1] >= 1 and flag == False:  # Marche chauffage et oneshot 
                reg_c.daily_save(current_time[0], current_time[7])
                flag = True
        elif current_time[4] != 1 :
            flag = False
# Calcul temps de cycle (ms)
        pycom.rgbled(0x000000)              # Eteint LED
        time.sleep(1.1)
        t_cycle = time.ticks_diff(start_t, time.ticks_ms())
        if DEBUG : print ('Temps de cycle : ', t_cycle,  ' ms')
        if DEBUG : print ("Date Heure locale", time.localtime())

# Pour relance watchdog
        if WATCH_DOG :
            wdog.feed()

