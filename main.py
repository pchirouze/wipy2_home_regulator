#/usr/bin/python3
# -*- coding: utf-8 -*-
''' ------------------------------------------------------------------------------------------------
Regulation chauffage eau plancher chauffant
    * Acquisition thermometres  DS18x20 (Pin 22)
        T1 : Température extérieure
        T2 : Température ambiante
        T3 : Température cuve solaire niveau interméiaire
        T4 : Température sortie vanne 3V
        T5 : Temperature sortie chaudiere
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

import json
#import sys
import time

import _thread
import machine
import onewire
import pycom
from machine import RTC, UART, Pin, Timer, WDT
from network import WLAN
from PID import PID
from umqtt import MQTTClient

# Constantes et configuration
#-------------------------- DEBUG ---------------------------
DEBUG = True
#DEBUG = False
#------------------ Emulation compteur EDF ------------------
SIMU = const(0)
#SIMU=const(1) 
#------------------- Watchdog -------------------------------
#WATCH_DOG = False
WATCH_DOG = True
STX = b'\x02'
ETX = b'\x03'
ON = const(1)              # Pour activer sorties logiques
OFF = const(0)
NBTHERMO = const(5) # Nombre de thermometres OneWire
# Trame d'emission simulation connexion compteur (téléinfo edf)
trame_edf = STX + \
    b'\nADCO 123456789012 $\r' + \
    b'\nOPTARIF HC.. $\r' + \
    b'\nISOUSC 45 $\r' + \
    b'\nHCHC 12345678 $\r' + \
    b'\nHCHP 12345678 $\r' + \
    b'\nPTEC HP.. $\r' + \
    b'\nIINST 012 $\r' + \
    b'\nADPS 123 $\r' + \
    b'\nIMAX 123 $\r' + \
    b'\nPAPP 12345 $\r' + \
    b'\nHHPHC 1 $\r' + \
    b'\nMOTDETAT 123456 $\r' + \
    ETX

# ------------------ Constantes ------------------------------------
#
# Identifiant interne capteur a changé si changement de capteur(en decimal 5 derniers digits)
# THERMOMETRES = {27702:'T1', 28196:'T2', 29859:'T3', 27423:'T4', 23570:'T5'}

#----------- Parametres par defaut pour creation fichiers ----------------------------
# T eau fonction de T ext, Consigne ambiante et ecart consigne ambiante - T ambianteregul_chauffe.py
# Parametres pour calcul loi d'eau lineaire par segment : 
# (offset(°C), (t_max_zon1(°C), pente1), (t_max2(°C),_zon2 pente2), ...)
param_cons = (24, (10, 0.5), (20, 0.45), (30, 0.42), (40, 0.40))

# Parametres pour regulation vanne
# (T cuve mini utilisable(°C), Bande morte regul(°C), t(s) pulse+/-, t(s) attente, t(s) ouverture 0-100%)
param_vanne = (26.0, 0.5, 5, 20, 60)

# Parametres pour regulation chaudiere electrique
# Kp,  Ki,  Kd, temps de cycle(s), max integrateur(°C)
param_chaudiere = (1.0, 0.1, 0.2, 10,  3)

# Parametres gestion commande electrique,
# (Seuils Cde résistances 1, 2 ,3, 4, courant(A), tension(V) unitaire par résistance)
param_electrique = (0.2,  0.3,  0.45, 0.6,  7.15,  230)
# Parametres de fonctionnement
# (Consigne T amb,(°C), Marche(1) Arret(0) chauffage)
param_fonct = [20.0, 1]
# -------------------------  Definitions ports entrées et sorties
p_circu = 'P19'                 # Cde circulateur
p_v3v_p = 'P20'                 # Cde + vanne 3 voies
p_v3v_m = 'P21'                 # Cde - vanne 3 voies
p_bus_ow = 'P22'                # bus OneWire pour DS18X20
p_R1 = 'P5'                     # Cde resistance R1
p_R2 = 'P6'                     # Cde resistance R2
p_R3 = 'P7'                     # Cde resistance R3
p_R4 = 'P8'                     # Cde resistance R4
p_hc = 'P9'                     # Heures creuses

# WIFI ID , PWD, MQTT broker
SSID='freebox_PC'
PWID='parapente'
MQTT_server="iot.eclipse.org"

# Thread reception téléinformation compteur EDF

def edf_recv(serial):
    ''' Docstring de la fonction '''
    global dic_edf, new_lec
    encours = False
    while True:
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
            elif encours is True:
#                print('ETX')
                mes+= car
                encours=False
###                serial.readall()
                tabl=[item.split(' ') for item in mes.decode().strip('\n\r\x02\x03').split('\r\n')]
                dic=dict([[item[0], item[1]] for item in tabl if(len(item)==3)])
                lock.acquire()
                dic_edf=dic.copy()
                new_lec=True
                lock.release()
# time.sleep(0.2)
                machine.idle()
#
# Calcul consigne température chauffage (loi d'eau lineaire par segment)
#
def calc_cons_eau(SetP_amb, T_amb, T_ext, params):
    ''' Calcul consigne T eau (loi d'eau linéaire par segment) '''
    ecart_t=SetP_amb - T_ext + (SetP_amb - T_amb)
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
    if marche==1 and t_amb < cons_amb + 1.5:
        pin_cde(0)
        return 1
    else :
        pin_cde(1)
        return 0
#
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

    def run(self, t_cons_eau, t_cuve, t_sortie_vanne, t_cycl, circulateur):
        ''' Active la régulation de la vanne '''
        if DEBUG :
            print('Etape : ',  self.etape,  'Tempo',  self.tempo)
        else :
            print('Etape regulation : ', self.etape)
        if self.etape == 0:         # Fermeture vanne position initiale de depart
            self.tempo = self.t_move
            self.pin_m(0)             # Collecteur ouvert 0 ---> active actionneur
            self.position = 0.0     # % ouverture vanne
            self.etape = 3

        if self.etape == 1:         # Controle regulation vanne
            if circulateur == 1:     # Chauffage On
                if t_cuve > self.t_cuve_min :
                    if t_cons_eau > t_cuve :
                        self.tempo=self.t_move           # Ouverture vanne a 100 %
                        self.pin_p(0)                             # Collecteur ouvert 0 ---> active actionneur
                        self.position = 100.0                # % ouverture vanne
                        self.etape = 3
                    elif t_sortie_vanne < t_cons_eau - self.deadband:
                        self.tempo = self.t_pulse
                        self.pin_p(0)
                        print('Pulse +')
                        if self.position < 100:
                            self.position += (self.t_pulse / self.t_move) * 100
                        else:
                            self.position = 100.0
                        self.etape = 2
                    elif t_sortie_vanne > t_cons_eau + self.deadband :
                        self.tempo = self.t_pulse
                        self.pin_m(0)
                        print('Pulse -')
                        if self.position > 0:
                            self.position -= (self.t_pulse / self.t_move ) * 100
                        else:
                            self.position = 0.0
                        self.etape = 2
                else:
                    self.tempo = self.t_move
                    self.pin_m(0)             # Collecteur ouvert 0 ---> active actionneur
                    self.position = 0.0     # % ouverture vanne
                    self.etape = 3

        if self.etape == 2:         # Gestion durée pulse +/- de correction
            if self.tempo > 0 :
                self.tempo -= t_cycl
            else:
                self.pin_p(1)
                self.pin_m(1)
                self.tempo = self.t_wait
                self.etape = 3

        if self.etape == 3:         # Attente après correction ou positionnement
            if self.tempo > 0 :
                self.tempo -= t_cycl
            else:
                self.pin_p(1)
                self.pin_m(1)
                self.etape = 1

    def get_pos_vanne(self):
        ''' Get position vanne regulation '''
        return self.position

#
# Regulation chaudiere si solaire insuffisant
#
class reg_chaudiere(PID):
    ''' Classe régulation chaudiere electrique '''
    def __init__(self,params):
        PID.__init__(self)
        self.puissance = 0
        self.output_reg=0
        self.setSampleTime(params[3])
        self.setWindup(params[4])


    def run(self, t_cons_eau,  t_sortie_chaudiere, circulateur,  params):
        ''' Activation regulation '''
        if circulateur ==1 :
            self.setSetpoint(t_cons_eau)
            self.setKp(params[0])
            self.setKi(params[1])
            self.setKd(params[2])
            self.update(t_sortie_chaudiere)             # PID calculation
            self.output_reg= self.output
        else:
            self.output_reg=0
        return self.output_reg

    def get_outputReg(self):
        ''' Lecture données sorties régulation '''
        return self.output_reg

#
# Gestion delestage electrique et calcul puissance de chauffage en heures creuses et en heures pleines
#
class  ges_elec():
    ''' Lecture compteur EDF '''
    def __init__(self, pin_r1,  pin_r2,  pin_r3,  pin_r4,  pin_hc):
        self.p_r = []
        self.p_r.append( pin_r1)
        self.p_r[0].init(mode=pin_r1.OUT)
        self.p_r.append( pin_r2)
        self.p_r[1].init(mode=pin_r2.OUT)
        self.p_r.append( pin_r3)
        self.p_r[2].init(mode=pin_r3.OUT)
        self.p_r.append( pin_r4)
        self.p_r[3].init(mode=pin_r4.OUT)
        self.pin_hc = pin_hc
        self.pin_hc.init(mode = pin_hc.OUT)
        self.nbr = 0
        self.kw_hc = 0
        self.kw_hp = 0
        self.puissance = 0.0

    def run(self, out_reg, data_edf, t_cycl,  params):
    	''' Docstring here '''
        self.output_reg = out_reg
#
# Calcul du courant disponible pour le chauffage (delestage)
        try:
            self.iinst= int(data_edf['IINST'])
            self.imax= int(data_edf['ISOUSC'])
            error = 0
        except :
            error = 1
            self.iinst =0
            self.imax = 45
# Calcul le nombre de resistances actionnables
        nbr_max = (self.imax - self.iinst) / params[4]
# Determine le nombre de resistances souhaité par le PID
        if self.output_reg < params[0]:
            self.nbR = 0
        elif self.output_reg  < params[1]:
            self.nbR = 1
        elif self.output_reg  < params[2]:
            self.nbR = 2
        elif self.output_reg  < params[3]:
            self.nbR = 3
        else:
            self.nbR = 4
# Pilotage des sorties de commandes resitances chaudiere
        if self.nbR > nbr_max:
            self.nbR = nbr_max
        for i in range(4): self.p_r[i].value(OFF)             #Reset sorties
        for i in range(self.nbR):  self.p_r[i].value(ON)   #Set sorties cde resistances
        self.puissance = self.nbR * params[4] * params[5]
        try :
            if data_edf['PTEC'] == 'HC..':
                self.kw_hc += self.puissance * t_cycl / 3600000  # conversions en w/h
                self.pin_hc(0)
            elif data_edf['PTEC'] == 'HP..':
                self.kw_hp += self.puissance * t_cycl / 3600000  # conversions en w/h
                self.pin_hc(1)
            error = 0
        except:
            error = 1
            self.kw_hp += self.puissance * t_cycl / 3600000  # conversions en w/h
            self.pin_hc(0)      # Si defaut maintenu sur plusieurs jours force en heures creuses
        return error

    def get_power(self):
    	''' Docstring here '''
        return self.puissance

    def get_energie(self):
    	''' Docstring here '''
        return self.kw_hc,  self.kw_hp
#
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
            param_fonct[1] = 1
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
    global param_fonct,  param_cons,  param_vanne,  param_chaudiere,  param_electrique, thermometres
    try:
        f=open('p_cons.dat', 'r')
        param_cons=json.loads(f.read())
    except:
        print('Erreur lecture fichier parametres')
    finally:
        #Cree fichier parametres par defaut
        f=open('p_cons.dat','w')
        f.write(json.dumps(param_cons))
        f.close()

    try:
        f=open('p_vanne.dat', 'r')
        param_vanne=json.loads(f.read())
    except:
        print('Erreur lecture fichier parametres')
    finally:
        #Cree fichier parametres par defaut
        f=open('p_vanne.dat','w')
        f.write(json.dumps(param_vanne))
        f.close()

    try:
        f=open('p_chaud.dat', 'r')
        param_chaudiere=json.loads(f.read())
    except:
        print('Erreur lecture fichier parametres')
    finally:
        #Cree fichier parametres par defaut
        f=open('p_chaud.dat','w')
        f.write(json.dumps(param_chaudiere))
        f.close()

    try:
        f=open('p_elec.dat', 'r')
        param_electrique=json.loads(f.read())
    except:
        print('Erreur lecture fichier parametres')
    finally:
        #Cree fichier parametres par defaut
        f=open('p_elec.dat','w')
        f.write(json.dumps(param_electrique))
        f.close()

    try:
        f=open('p_fonct.dat', 'r')
        param_fonct=json.loads(f.read())
    except:
        print('Erreur lecture fichier parametres')
    finally:
        #Cree fichier parametres
        f=open('p_fonct.dat', 'w')
        f.write(json.dumps(param_fonct))
        f.close()
    
    # Lecture fichiers affections thermometres
    try:
        f=open('thermo.dat', 'r')
        data=f.read()
        thermometres = json.loads(data)
    except:
        print('Erreur lecture fichier thermometres')
        thermometres = {}
        dev = ds.roms
        if len(dev) == NBTHERMO:
    # Affectation des thermometres et enregistrement (converti ID en int: bug bytearray en json)
            for i,idn in enumerate(dev):
                thermometres['T'+ chr(0x31+i)] = int.from_bytes(idn,'little') 
            f=open('thermo.dat','w')
            print(thermometres)
            f.write(json.dumps(thermometres))
        else:
            print('Seulement ', len(dev), 'thermometres detectés sur ', NBTHERMO )
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
# Instances  regulation vanne, chaudiere et gestion electrique
reg_v = regul_vanne(Pin(p_v3v_p),  Pin(p_v3v_m),  param_vanne)
reg_c = reg_chaudiere(param_chaudiere)
c_elec = ges_elec(Pin(p_R1),  Pin(p_R2),  Pin(p_R3),  Pin(p_R4),  Pin(p_hc))
# Lecture fichiers parametres
lecture_fichiers()
etape_wifi = 0
mes_send = False
cpt_send=0
data_cpt={}
data_reel={}
alive = False
#
#---------------------------------- Main loop --------------------------------------------
#
pycom.heartbeat(False)
all_t_read = 0
while True:
    pycom.rgbled(0x080000)
# Init watchdog
    if WATCH_DOG :
        wdog = WDT(timeout=15000)
#Lecture thermometres OneWire (Raffraichi un thermometre par boucle)
    for key in thermometres:
        start_t = time.ticks_ms()    
        idt= thermometres[key].to_bytes(8,'little')
        ds.start_convertion(idt)
        time.sleep(0.7)
        t_lue = ds.read_temp_async(idt)/100.0
        if t_lue >=4095 :                   # ds18 debranché valeur = 4095.xx
            print('Defaut capteur ou non enregistre',key )
            temp[key]=0.0
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
            else:
                print('Defaut lecture teleinfo EDF')
            new_lec=False
            lock.release()
            if DEBUG :        print('Compteur EDF : ',  data_cpt)
    # Calcul consigne  temp eau chauffage
            cons_eau= calc_cons_eau(param_fonct[0], temp['T1'],  temp['T2'], param_cons )
            if DEBUG :  print('Temp. consigne eau : ',  cons_eau)
    # Controle circulateur
            etat_circ = cnt_circulateur(param_fonct[0],  temp['T2'], s_circul, param_fonct[1] )
            if DEBUG : print('Cde circulateur : ', etat_circ)
    # Regulation vanne 3 voie sur circuit solaire
            reg_v.run(cons_eau,  temp['T3'],  temp['T4'],  t_cycle,  etat_circ)
            position=reg_v.get_pos_vanne()
            if DEBUG : print('Ouverture vanne : ',  position,  ' %')
    # Regulation chaudiere
            reg_c.run(cons_eau, temp['T5'], etat_circ,  param_chaudiere)
            if DEBUG : print('Sortie PID chaudiere : ',  reg_c.get_outputReg())
    # Gestion electrique  (delestage)
            erreur = c_elec.run(reg_c.get_outputReg(),  data_cpt, t_cycle,   param_electrique)
    # Puissance chauffage(W),  energie cumulé en HC et HP (kWh)
            if DEBUG :
                print('Puissance chauffe : ', c_elec.get_power(), ' W')
                print('Energie heures creuses : ',  c_elec.get_energie() [0] / 1000,  ' kWh')
                print('Energie heures pleines : ', c_elec.get_energie() [1] / 1000.0,  'kWh')

# Gestion protocole Telnet, FTP, MQTT en WiFI   print (wifi, mqtt_ok)
            if etape_wifi == 0:
                lswifi=[]
                wlan=WLAN(mode=WLAN.STA,antenna=WLAN.INT_ANT)
                try:
                    lswifi=wlan.scan()
                except:
                    print('Pas de connexion Wifi')
                for r in lswifi:
# freebox et signal > -80 dB
                    if r[0] == SSID and r[4] > -80 :
#                        wlan.ifconfig(config=('192.168.0.30', '255.255.255.0', '192.168.0.254', '212.27.40.240'))
                        wlan.ifconfig(config='dhcp')
                        wlan.connect(SSID, auth=(WLAN.WPA2, PWID), timeout=50)
                        time.sleep(2)
                        etape_wifi = 1

# Creation et initialisation protocole MQTT 
            if etape_wifi == 1:
                if wlan.isconnected(): 
                    print('Connecte WIFI : ',  wlan.ifconfig())
                    rtc=RTC()
                    rtc.ntp_sync("pool.ntp.org")
                    time.timezone(3600)
                    client =MQTTClient("chauffage",MQTT_server, port = 1883, keepalive=100)
                    try:
                        client.connect(clean_session=True)
                        client.set_callback(incoming_mess)
                        client.subscribe('/regchauf/cde', qos= 0)
                        client.subscribe('/regchauf/send', qos= 0)
                        client.subscribe('/regchauf/cons', qos= 0)
                        print('Connecte au serveur MQTT : ',  MQTT_server)
                        etape_wifi = 2
                    except:
                        print('MQTT connexion erreur')
# WIFI et MQTT Ok 
            if etape_wifi == 2:
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
                        data_reel['CHAU'] = reg_c.get_outputReg()
                        data_reel['ELEC'] = {'PW': c_elec.get_power(), 'CHC' : c_elec.get_energie()[0] / 1000, 'CHP' : c_elec.get_energie()[ 1] / 1000}
                        data_reel['FNCT'] = param_fonct
                        client.publish('/regchauf/mesur',json.dumps(data_reel))
                        if DEBUG : print('Publication mesures : ',  json.dumps(data_reel))
                    except:
                        client.disconnect()
                        print('MQTT publication erreur')
                        etape_wifi = 1
                
                alive = not alive
                client.publish('/regchauf/alive', bytes(str(alive), "utf8"))

            if DEBUG: print('Etape Wifi: ', etape_wifi) 
            #--------- Pour simulation liaison compteur Edf (jumper RX-TX loop)
            if SIMU == 1:  ser.write(trame_edf)
            #--------------------------------------------------------------

            time.sleep(0.7)
            # Calcul temps de cycle (ms)
            t_cycle=time.ticks_diff(start_t, time.ticks_ms())
            machine.idle()
            if DEBUG : print ('Temps de cycle : ', t_cycle,  ' ms')

    # Pour relance watchdog
        if WATCH_DOG :
            wdog.feed()

