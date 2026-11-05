#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver servo = Adafruit_PWMServoDriver();

#define SERVOMIN 120
#define SERVOMAX 470
#define SERVO_FREQ 50

// Posizione ATTUALE di ogni canale (cambia a ogni movimento) - indice = canale
// Valore iniziale = POSA SEDUTA (quella del vecchio seduto(); il canale 0 resta a 70).
// All'avvio, setup() manda questi valori ai servo: il robot deve essere gia' seduto.
int servoPos[16] = {
  70, 65, 130, 170, 170, 60, 60, 70, 150, 20, 135, 90, 180, 180, 40, 160
};

// ============================================================================
// LEGENDA CANALI:  canale nome -> cosa fa 0 e cosa fa 180 - posizione base
// Fonte: i tuoi appunti del vecchio seduto(). "?" = da compilare tu.
// ============================================================================
//  0 polso dx            -> 0 = ? - 180 = ? - base 70
//  1 spalla frontale dx  -> 0 = ? - 180 = ? - base 75
//  2 spalla frontale sx  -> 0 = giù ↓ - 180 = su ↑ - base 110
//  3 spalla laterale dx  -> 0 = ? - 180 = ? - base 65
//  4 spalla laterale sx  -> 0 = fuori - 180 = dentro - base 90
//  5 spalla posteriore dx -> 0 = fuori - 180 = dentro - base 80
//  6 spalla posteriore sx -> 0 = fuori - 180 = dentro - base 85
//  7 busto               -> 0 = ? - 180 = ? - base 90
//  8 inguine (sx/dx?)    -> 0 = ? - 180 = ? - base 30
//  9 inguine (sx/dx?)    -> 0 = fuori - 180 = dentro - base 150
// 10 anca dx             -> 0 = fuori - 180 = dentro - base 90
// 11 anca sx             -> 0 = dentro - 180 = fuori - base 140
// 12 coscia dx           -> 0 = giù ↓ - 180 = su ↑ - base 90
// 13 coscia sx           -> 0 = giù ↓ - 180 = su ↑ - base 90
// 14 ginocchio dx        -> 0 = giù ↓ - 180 = su ↑ - base 90
// 15 ginocchio sx        -> 0 = su ↑ - 180 = giù ↓ - base 95

// Posizione BASE (inPiedi)
const int posizioniSicure[16] = {
  70,  // canale 0  - polso dx
  75,  // canale 1  - spalla frontale dx
  110, // canale 2  - spalla frontale sx
  65,  // canale 3  - spalla laterale dx
  90,  // canale 4  - spalla laterale sx
  80,  // canale 5  - spalla posteriore dx
  85,  // canale 6  - spalla posteriore sx
  90,  // canale 7  - busto
  30,  // canale 8  - inguine (verificare sx/dx, vedi nota nel markdown)
  150, // canale 9  - inguine dx
  90,  // canale 10 - anca dx
  140, // canale 11 - anca sx
  90,  // canale 12 - coscia dx
  90,  // canale 13 - coscia sx
  90,  // canale 14 - ginocchio dx
  95   // canale 15 - ginocchio sx
};

int velocitaServo = 15; // ms tra un passo e l'altro nei movimenti graduali (comando 'm' per cambiarla)

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);

  servo.begin();
  servo.setOscillatorFrequency(27000000);
  servo.setPWMFreq(SERVO_FREQ);

  delay(100);

  // All'avvio i valori di servoPos (posa SEDUTA) vengono mandati ai servo, cosi' tutti
  // i canali hanno segnale e la posizione assunta coincide con quella reale.
  // ATTENZIONE: il robot DEVE essere gia' fisicamente seduto, altrimenti scatta.
  // Questo succede a OGNI reset dell'Arduino (anche aprendo il monitor seriale).
  for (uint8_t canale = 0; canale < 16; canale++) {
    muoviServo(canale, servoPos[canale]);
  }

  //preparaPose(); // calcola posaSeduto/posaInPiedi da posizioniSicure
}

void loop() {
  if (Serial.available() > 0) {
    String comando = Serial.readStringUntil(';');
    comando.trim();
    if (comando.length() > 0) gestisciComando(comando);
  }
}

// ============================================================================
// MOVIMENTO SERVO
// ============================================================================

// Sposta un servo istantaneamente alla posizione target (nessun controllo di velocita')
void muoviServo(uint8_t canale, int angolo) {
  angolo = constrain(angolo, 0, 180);
  int pwm = map(angolo, 0, 180, SERVOMIN, SERVOMAX);
  servo.setPWM(canale, 0, pwm);
  servoPos[canale] = angolo;
}

// Sposta un servo gradualmente, un grado alla volta, dalla posizione attuale al target.
// Al momento nessun comando la richiama: tenuta come funzione di base.
void reachGoal(uint8_t canale, int target, int velocita) {
  target = constrain(target, 0, 180);
  int passo = (target > servoPos[canale]) ? 1 : -1;

  while (servoPos[canale] != target) {
    muoviServo(canale, servoPos[canale] + passo);
    delay(velocita);
  }
}

// Sposta piu' servi insieme, sincronizzati: tutti arrivano al target nello stesso momento
void reachGoals(const uint8_t *canali, const int *target, uint8_t n, int velocita) {
  static int posPartenza[16];
  int passiMax = 0;

  for (uint8_t i = 0; i < n; i++) {
    posPartenza[i] = servoPos[canali[i]];
    int p = abs(constrain(target[i], 0, 180) - posPartenza[i]);
    if (p > passiMax) passiMax = p;
  }
  if (passiMax == 0) return;

  for (int step = 1; step <= passiMax; step++) {
    for (uint8_t i = 0; i < n; i++) {
      int t = constrain(target[i], 0, 180);
      int pos = posPartenza[i] + (long)(t - posPartenza[i]) * step / passiMax;
      muoviServo(canali[i], pos);
    }
    delay(velocita);
  }
}

// Sposta TUTTI i 16 canali verso una "posa" (array di 16 target), sincronizzati.
// E' un guscio sottile attorno a reachGoals: nessuna logica di movimento nuova,
// solo un modo comodo per richiamare pose a corpo intero (in piedi, seduto, ecc.)
void vaiAPosa(const int *posa, int velocita) {
  uint8_t tutti[16];
  for (uint8_t i = 0; i < 16; i++) tutti[i] = i;
  reachGoals(tutti, posa, 16, velocita);
}

// ============================================================================
// GESTI TEMPORANEI (portati dal file di test) - richiamabili con p<numero>;
// ============================================================================
// Ogni gesto e' una funzione con i suoi due array (canali e target).
// Si modificano qui, al volo. Quando ci sara' Python
// questa sezione sparira': sara' Python a mandare direttamente i comandi M.
// I canali di ogni gesto ora si muovono INSIEME e gradualmente (prima erano
// scatti uno dopo l'altro con delay). Partire sempre dalla posa base.

void apri() {                                  // p1
  const uint8_t canali[] = {9, 8, 11, 10, 12, 13};
  const int target[]     = {10, 180, 90, 135, 95, 95} ;
  reachGoals(canali, target, 2, velocitaServo);
}

void chiudi() {                                // p2
  const uint8_t canali[] = {9, 8};
  const int target[]     = {40, 140};
  reachGoals(canali, target, 2, velocitaServo);
}

void posizioneBracciaLegate() {                // p3
  const uint8_t canali[] = {5, 6, 3, 1};
  const int target[]     = {55, 65, 160, 120};
  reachGoals(canali, target, 4, velocitaServo);
}

void spostaBraccio() {                         // p4
  const uint8_t canali[] = {3, 1};
  const int target[]     = {150, 60};
  reachGoals(canali, target, 2, velocitaServo);
}

void alzaBraccio() {                           // p5
  const uint8_t canali[] = {5, 1};
  const int target[]     = {90, 180};
  reachGoals(canali, target, 2, velocitaServo);
}

// p6 - posa seduta, valori del vecchio seduto() (provati sul robot).
// Il canale 0 non c'era nel vecchio seduto(): qui non viene toccato.
// Muove 15 canali insieme dalla posizione attuale: partire dalla posa base,
// robot sospeso e sotto supervisione la prima volta.
// Legenda dei versi: vedi LEGENDA CANALI in alto.
void seduto() {
  const uint8_t canali[] = {15,  14, 13,  12, 11, 10, 9,  8,   7, 6,  5,  4,   3,   2,   1};
  const int target[]     = {160, 40, 180, 180, 90, 135, 20, 150, 70, 60, 60, 170, 170, 130, 65};
  reachGoals(canali, target, 15, velocitaServo);
}

// Esegue il gesto numero n. Restituisce false se il numero non esiste.
bool eseguiGesto(int n) {
  switch (n) {
    case 1: apri(); break;
    case 2: chiudi(); break;
    case 3: posizioneBracciaLegate(); break;
    case 4: spostaBraccio(); break;
    case 5: alzaBraccio(); break;
    case 6: seduto(); break;
    default: return false;
  }
  return true;
}

// ============================================================================
// POSA: SEDUTO / IN PIEDI   (S; e A; DISATTIVATI per ora - vedi gestisciComando)
// ============================================================================
// ATTENZIONE - valori PLACEHOLDER: verso (+/-) ed entita' del movimento non
// sono ancora stati verificati fisicamente. Da tarare quando il robot e'
// di nuovo disponibile, SEMPRE sotto supervisione la prima volta.
// Strategia scelta: mentre ginocchia+coscia si piegano, le spalle frontali
// (braccia) vanno in avanti come contrappeso per aiutare l'equilibrio.
// Anca, inguine e busto per ora restano fermi, per limitare le variabili
// del primo test.

const int DELTA_GINOCCHIO_SEDUTO = -30; // PLACEHOLDER
const int DELTA_COSCIA_SEDUTO    = -20; // PLACEHOLDER
const int DELTA_BRACCIO_SEDUTO   =  40; // PLACEHOLDER (contrappeso in avanti)

int posaSeduto[16];
int posaInPiedi[16];

void preparaPose() {
  for (uint8_t i = 0; i < 16; i++) {
    posaInPiedi[i] = posizioniSicure[i];
    posaSeduto[i]  = posizioniSicure[i];
  }

  posaSeduto[15] += DELTA_GINOCCHIO_SEDUTO; // ginocchio sx
  posaSeduto[14] += DELTA_GINOCCHIO_SEDUTO; // ginocchio dx
  posaSeduto[13] += DELTA_COSCIA_SEDUTO;    // coscia sx
  posaSeduto[12] += DELTA_COSCIA_SEDUTO;    // coscia dx
  posaSeduto[2]  += DELTA_BRACCIO_SEDUTO;   // spalla frontale sx (contrappeso)
  posaSeduto[1]  += DELTA_BRACCIO_SEDUTO;   // spalla frontale dx (contrappeso)
}

// Sedersi in 2 fasi: prima meta' flessione (pausa per controllare), poi il resto.
// La meta' strada si calcola da sola, non serve una terza posa scritta a mano.
void siediti() {
  int metaStrada[16];
  for (uint8_t i = 0; i < 16; i++) {
    metaStrada[i] = posizioniSicure[i] + (posaSeduto[i] - posizioniSicure[i]) / 2;
  }

  vaiAPosa(metaStrada, velocitaServo);
  delay(300); // pausa di sicurezza tra le due fasi
  vaiAPosa(posaSeduto, velocitaServo);
}

// Alzarsi: stessa logica, all'incontrario
void alzati() {
  int metaStrada[16];
  for (uint8_t i = 0; i < 16; i++) {
    metaStrada[i] = posaSeduto[i] + (posizioniSicure[i] - posaSeduto[i]) / 2;
  }

  vaiAPosa(metaStrada, velocitaServo);
  delay(300);
  vaiAPosa(posizioniSicure, velocitaServo);
}

// ============================================================================
// COMANDI DA SERIALE (terminale Arduino IDE, o in futuro Python)
// ============================================================================
// Formato: 1 carattere comando + eventuali cifre + ';'
//   M<canale>:<target>,<canale>:<target>...;  muovi piu' canali insieme,
//                                              sincronizzati         es. M15:60,11:100,9:120;
//   m<velocita 1-3 cifre>;                  imposta la velocita    es. m020;   (minuscola, non 'M')
//   p<numero>;                              gesto temporaneo       es. p1;  (1 apri, 2 chiudi,
//                                              3 braccia legate, 4 sposta braccio, 5 alza braccio, 6 seduto)
//   h;                                      torna alla base sicura
//   r;                                      stampa la posizione attuale di tutti i canali
//   S;  A;                                  siediti / alzati: DISATTIVATI per ora
// Convenzione: lettere MAIUSCOLE = pose a corpo intero, minuscole = comandi
// su un singolo canale o di configurazione. Da rispettare anche in futuro.

void gestisciComando(const String& s) {
  char cmd = s[0];

  switch (cmd) {

    case 'M': {
      String resto = s.substring(1); // es. "15:60,11:100,9:120"
      uint8_t canali[16];
      int target[16];
      uint8_t n = 0;
      int pos = 0;

      while (pos < (int)resto.length() && n < 16) {
        int virgola = resto.indexOf(',', pos);
        String coppia = (virgola == -1) ? resto.substring(pos) : resto.substring(pos, virgola);

        int duePunti = coppia.indexOf(':');
        if (duePunti > 0) {
          uint8_t canale = coppia.substring(0, duePunti).toInt();
          int val = coppia.substring(duePunti + 1).toInt();
          if (canale < 16) {
            canali[n] = canale;
            target[n] = val;
            n++;
          }
        }

        if (virgola == -1) break;
        pos = virgola + 1;
      }

      if (n > 0) {
        reachGoals(canali, target, n, velocitaServo);
        inviaConferma('M', n);
      }
      break;
    }

    case 'm':
      velocitaServo = constrain(s.substring(1).toInt(), 1, 100);
      inviaConferma('m', velocitaServo);
      break;

    case 'p': {
      int numero = s.substring(1).toInt();
      if (eseguiGesto(numero)) {
        inviaConferma('p', numero);
      } else {
        Serial.println("p: gesto sconosciuto");
      }
      break;
    }

    case 'h': {
      uint8_t tutti[16];
      for (uint8_t i = 0; i < 16; i++) tutti[i] = i;
      reachGoals(tutti, posizioniSicure, 16, velocitaServo);
      inviaConferma('h', 1);
      break;
    }

    // DISATTIVATI: posaSeduto e' tutta a zero finche' preparaPose() resta
    // commentata in setup(): 'S' manderebbe tutti i servo a 0 gradi.
    // Riattivare solo dopo aver sistemato le pose.
    /*
    case 'S':
      siediti();
      inviaConferma('S', 1);
      break;

    case 'A':
      alzati();
      inviaConferma('A', 1);
      break;
    */

    case 'r':
      for (uint8_t i = 0; i < 16; i++) {
        Serial.print(i);
        Serial.print(':');
        Serial.print(servoPos[i]);
        Serial.print(' ');
      }
      Serial.println();
      break;
  }
}

// Conferma via seriale dopo ogni comando (utile leggendola da terminale o da Python)
void inviaConferma(char comando, int valore) {
  Serial.print(comando);
  Serial.print(':');
  Serial.println(valore);
}
