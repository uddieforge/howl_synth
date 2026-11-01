// ============================================================
//  4 VCO SuperSaw (2 saws/VCO) + LFO + Enveloppes + Filtre + Bitcrusher
//  Arduino Nano + Mozzi 2.0 — Sortie D9
//
//  LED D2  : Menu 1
//  LED D4  : Menu 2 + forme LFO : fixe=Sin | lent=Pulse | rapide=S&H
//  LED D5  : Menu 3 + mode env : fixe=Hold | lent=Env | rapide=Loop
//  LED D6  : Menu 4 (filtre)
//  LED D7  : Menu 5 bitcrusher : fixe=OFF | clignote=ON
//  LED D13 : cible LFO (permanent, tous menus) :
//            eteinte=Vol | lent=Pitch | moyen=Filtre | rapide=Bitcrush
//
//  D8  : avance menu
//  D3  : randomisation
//  D10 : avance sous-mode (menus 2 et 3)
//  D11 : cycle cible LFO vol/pitch/filtre/bitcrush (menu 2)
//        trig enveloppe (menu 3 mode Env)
//  D12 : toggle bitcrusher ON/OFF (menu 5)
//
//  MENU 1 : A0-A3 pitch VCO1-4 / A4-A7 volume max VCO1-4
//  MENU 2 : A0-A3 tempo LFO / A4-A7 depth LFO
//  MENU 3 : A0-A3 attack / A4-A7 release
//  MENU 4 : A0-A3 cutoff filtre VCO1-4 / A4-A7 resonance VCO1-4
//  MENU 5 : A0-A3 bit reduction / A4-A7 sample rate reduction
// ============================================================

#define MOZZI_CONTROL_RATE 32
#include <Mozzi.h>
#include <Oscil.h>
#include <tables/saw2048_int8.h>
#include <tables/sin2048_int8.h>
#include <tables/square_no_alias_2048_int8.h>

// ── Broches ───────────────────────────────────────────────
#define BTN_MENU    8
#define BTN_RAND    3
#define BTN_SUBMENU 10
#define BTN_TRIG    11
#define BTN_D       12
#define LED_LFO_TARGET 13  // indicateur cible LFO (permanent)

// ── LEDs ──────────────────────────────────────────────────
#define NUM_MENUS 5
const uint8_t MENU_LEDS[NUM_MENUS] = { 2, 4, 5, 6, 7 };
#define BLINK_SLOW_MS  500
#define BLINK_MED_MS   250
#define BLINK_FAST_MS  120

// ── Prototypes ────────────────────────────────────────────
void tickLfoMod();
int32_t applyBitcrush(int32_t s, uint8_t bits_amt, uint8_t rate_amt, uint8_t v);

// ── Variables globales anticipees (utilisees dans updateMenuLeds) ─
bool     bc_active      = false;
bool     btn_d12_prev   = false;
// Toggle unison (D11 menu 1) — VCO2/3/4 suivent VCO1
bool unison_active = false;

// Toggle mute global (D12 menu 1)
bool mute_global  = false;
uint32_t d12_lockout_at = 0;
uint8_t  bc_bits[4]     = { 0, 0, 0, 0 };
uint8_t  bc_rate[4]     = { 0, 0, 0, 0 };
int32_t  bc_held[4]     = { 0, 0, 0, 0 };
uint8_t  bc_counter[4]  = { 0, 0, 0, 0 };

// ──────────────────────────────────────────────────────────
//  SOUS-MODES
// ──────────────────────────────────────────────────────────

typedef enum { LFO_SIN = 0, LFO_PULSE, LFO_SH }             LfoShape;
typedef enum { SUBMODE_HOLD = 0, SUBMODE_ENV, SUBMODE_ENVLOOP } SubMode;
typedef enum { LFO_TARGET_VOL=0, LFO_TARGET_PITCH,
               LFO_TARGET_FILTER, LFO_TARGET_CRUSH }         LfoTarget;

LfoShape  lfo_shape     = LFO_SIN;
SubMode   env_submode   = SUBMODE_HOLD;
LfoTarget lfo_target    = LFO_TARGET_VOL;

bool     btn_sub_prev    = false;
uint32_t sub_lockout_at  = 0;
bool     btn_trig_prev   = false;
uint32_t trig_lockout_at = 0;

// ──────────────────────────────────────────────────────────
//  MENUS
// ──────────────────────────────────────────────────────────

uint8_t  current_menu    = 0;
bool     btn_menu_prev   = false;
uint32_t menu_lockout_at = 0;
#define  MENU_LOCKOUT_MS 200

void updateMenuLeds(uint32_t now) {
  // D2-D7 : LEDs de menu simples
  digitalWrite(MENU_LEDS[0], current_menu == 0 ? HIGH : LOW);
  digitalWrite(MENU_LEDS[1], current_menu == 1 ? HIGH : LOW);
  digitalWrite(MENU_LEDS[2], current_menu == 2 ? HIGH : LOW);
  digitalWrite(MENU_LEDS[3], current_menu == 3 ? HIGH : LOW);
  digitalWrite(MENU_LEDS[4], current_menu == 4 ? HIGH : LOW);

  // D13 : usage contextuel selon le menu actif
  if (current_menu == 1) {
    // Menu 2 : indique la cible LFO
    switch (lfo_target) {
      case LFO_TARGET_VOL:
        digitalWrite(LED_LFO_TARGET, LOW); break;
      case LFO_TARGET_PITCH:
        digitalWrite(LED_LFO_TARGET, ((now/BLINK_SLOW_MS)%2)==0?HIGH:LOW); break;
      case LFO_TARGET_FILTER:
        digitalWrite(LED_LFO_TARGET, ((now/BLINK_MED_MS)%2)==0?HIGH:LOW); break;
      case LFO_TARGET_CRUSH:
        digitalWrite(LED_LFO_TARGET, ((now/BLINK_FAST_MS)%2)==0?HIGH:LOW); break;
    }
  } else if (current_menu == 2) {
    // Menu 3 : indique le sous-mode enveloppe
    switch (env_submode) {
      case SUBMODE_HOLD:
        digitalWrite(LED_LFO_TARGET, HIGH); break;        // fixe = Hold
      case SUBMODE_ENV:
        digitalWrite(LED_LFO_TARGET, ((now/BLINK_SLOW_MS)%2)==0?HIGH:LOW); break; // lent = Env
      case SUBMODE_ENVLOOP:
        digitalWrite(LED_LFO_TARGET, ((now/BLINK_FAST_MS)%2)==0?HIGH:LOW); break; // rapide = Loop
    }
  } else if (current_menu == 4) {
    // Menu 5 : indique si le bitcrusher est actif
    digitalWrite(LED_LFO_TARGET, bc_active ? HIGH : LOW);
  } else {
    digitalWrite(LED_LFO_TARGET, LOW);
  }
}

// ──────────────────────────────────────────────────────────
//  POT CATCHUP
// ──────────────────────────────────────────────────────────

#define NUM_POTS 8
int  pot_ref[NUM_POTS];
bool pot_caught[NUM_POTS];
int  pot_applied[NUM_MENUS][NUM_POTS];

void catchupInit() {
  for (uint8_t m = 0; m < NUM_MENUS; m++)
    for (uint8_t i = 0; i < NUM_POTS; i++)
      pot_applied[m][i] = -1;
  for (uint8_t i = 0; i < NUM_POTS; i++) {
    pot_caught[i] = false;
    pot_ref[i]    = 512;
  }
}

void initCatchup() {
  for (uint8_t i = 0; i < NUM_POTS; i++) {
    int last      = pot_applied[current_menu][i];
    pot_ref[i]    = (last < 0) ? mozziAnalogRead<10>(A0+i) : last;
    pot_caught[i] = false;
  }
  if (current_menu == 4)
    for (uint8_t v = 0; v < 4; v++) { bc_counter[v]=0; bc_held[v]=0; }
}

int potRead(uint8_t idx) {
  int raw = mozziAnalogRead<10>(A0 + idx);
  if (pot_caught[idx]) {
    pot_applied[current_menu][idx] = raw;
    return raw;
  }
  int delta = raw - pot_ref[idx];
  if (delta < 0) delta = -delta;
  if (delta <= 20) {
    pot_caught[idx] = true;
    pot_applied[current_menu][idx] = raw;
    return raw;
  }
  return -1;
}

// ──────────────────────────────────────────────────────────
//  PARAMETRES AUDIO GLOBAUX
// ──────────────────────────────────────────────────────────

float   base_freq[4]     = { 220.0f, 220.0f, 220.0f, 220.0f };
uint8_t vol_max[4]       = { 200, 200, 200, 200 };
float   lfo_vol_freq[4]  = { 1.0f, 1.0f, 1.0f, 1.0f };
uint8_t lfo_vol_depth[4] = { 0, 0, 0, 0 };
uint8_t vol_eff[4]       = { 200, 200, 200, 200 };
// Lissage vol pour eviter les clics (slew en updateAudio)
uint8_t vol_smooth[4]    = { 200, 200, 200, 200 };
int8_t  lfo_pitch_val[4] = { 0, 0, 0, 0 };
int8_t  lfo_mod_val[4]   = { 0, 0, 0, 0 };

// ──────────────────────────────────────────────────────────
//  FILTRE PASSE-BAS 2 POLES (Chamberlin State Variable)
//  Filtre SVF numerique, implementation entiere Q15
//
//  Equations :
//    low  = low + f * band
//    high = input - low - q * band
//    band = f * high + band
//    notch= high + low
//
//  f = 2 * sin(pi * fc / fs) ≈ 2 * pi * fc / fs  pour fc << fs
//  q = 1/resonance (0.5 = max resonance, 2.0 = pas de resonance)
//
//  On precalcule f et q dans updateControl (appelé à 64 Hz)
//  et on les applique dans updateAudio (16384 Hz).
//
//  Pour rester en entiers : on travaille en Q12
//  f_q12 = f * 4096, range 0..4096
//  q_q12 = q * 4096, range 2048..16384 (q de 0.5 a 4.0)
//
//  cutoff  : 0-255, mappe sur 20-8000 Hz (log)
//  resonance: 0-255, mappe sur q 2.0..0.6
// ──────────────────────────────────────────────────────────

uint8_t  filt_cut[4]  = { 255, 255, 255, 255 }; // 255=ouvert
uint8_t  filt_res[4]  = { 0,   0,   0,   0   }; // 0=pas de resonance
// Coefficients precalcules (updateControl)
int16_t  filt_f[4]    = { 4095, 4095, 4095, 4095 }; // Q12, 4095=bypass
int16_t  filt_q[4]    = { 8192, 8192, 8192, 8192 }; // Q12
// Etats internes du filtre (Q12, mis a jour dans updateAudio)
int32_t  svf_low[4]   = { 0, 0, 0, 0 };
int32_t  svf_band[4]  = { 0, 0, 0, 0 };
// Coefficient f effectif (peut etre module par le LFO)
int16_t  filt_f_eff[4] = { 4095, 4095, 4095, 4095 };

// Precalcule filt_f et filt_q depuis filt_cut et filt_res
// Appelé dans updateControl quand les pots changent
void computeFilterCoeffs(uint8_t v) {
  // Cutoff : mapping exponentiel sur 8 octaves (20 Hz -> 5000 Hz)
  // f = 2*pi*fc/fs, fs=16384 Hz
  // On precalcule filt_f en Q12 (x4096)
  // pot 0-255 -> fc 20-5000 Hz (expo)
  // fc = 20 * 2^(c/255 * 7.97)  -> 20..5000 Hz
  // f = 2*pi*fc/fs ~= fc/2605  pour fc << fs
  // filt_f = f * 4096 = fc * 4096 / 2605 = fc * 1.572
  // Approche en entiers : filt_f = 20 + c^2 * 61 / 255
  // (courbe parabolique, bonne approximation expo sur cette plage)
  uint8_t c = filt_cut[v];
  int32_t f = 4 + ((int32_t)c * c * 60) / 255; // 4..~15000 -> clamp 4095
  if (f > 4095) f = 4095;
  filt_f[v]     = (int16_t)f;
  filt_f_eff[v] = (int16_t)f;

  // Resonance : q va de 4096 (neutre, q=1.0) a 512 (forte, q=0.125)
  // q faible = resonance forte (pic accentue, peut auto-osciller)
  // q=4096 -> neutre, q=512 -> self-oscillation
  uint8_t r = filt_res[v];
  int32_t q = 4096 - ((int32_t)r * 3584 / 255); // 4096..512
  if (q < 512)  q = 512;
  filt_q[v] = (int16_t)q;
}

// Applique le filtre SVF sur un sample (Q0, range -256..254)
// Retourne la sortie passe-bas dans la meme plage
inline int32_t applyFilter(int32_t x, uint8_t v) {
  // Bypass seulement si cutoff completement ouvert ET pas de resonance
  if (filt_f[v] >= 4095 && filt_res[v] == 0) return x;

  // Mise a l'echelle Q12 pour la precision
  int32_t xq = x << 4; // Q4 suffit pour eviter overflow

  // SVF :
  int32_t f = filt_f_eff[v];
  int32_t q = filt_q[v];

  svf_low[v]  = svf_low[v]  + ((f * svf_band[v]) >> 12);
  int32_t hi  = xq - svf_low[v] - ((q * svf_band[v]) >> 12);
  svf_band[v] = ((f * hi) >> 12) + svf_band[v];

  // Clamp genereux : laisse la resonance s'exprimer
  if (svf_band[v] >  8192) svf_band[v] =  8192;
  if (svf_band[v] < -8192) svf_band[v] = -8192;
  if (svf_low[v]  >  8192) svf_low[v]  =  8192;
  if (svf_low[v]  < -8192) svf_low[v]  = -8192;

  // Sortie passe-bas, ramene en Q0
  return svf_low[v] >> 4;
}

// ──────────────────────────────────────────────────────────
//  VCO — 4 x 2 saws
// ──────────────────────────────────────────────────────────

Oscil<SAW2048_NUM_CELLS, AUDIO_RATE> vco1a(SAW2048_DATA);
Oscil<SAW2048_NUM_CELLS, AUDIO_RATE> vco1b(SAW2048_DATA);
Oscil<SAW2048_NUM_CELLS, AUDIO_RATE> vco2a(SAW2048_DATA);
Oscil<SAW2048_NUM_CELLS, AUDIO_RATE> vco2b(SAW2048_DATA);
Oscil<SAW2048_NUM_CELLS, AUDIO_RATE> vco3a(SAW2048_DATA);
Oscil<SAW2048_NUM_CELLS, AUDIO_RATE> vco3b(SAW2048_DATA);
Oscil<SAW2048_NUM_CELLS, AUDIO_RATE> vco4a(SAW2048_DATA);
Oscil<SAW2048_NUM_CELLS, AUDIO_RATE> vco4b(SAW2048_DATA);

// LFO pitch (tres lents, fixes)
Oscil<SIN2048_NUM_CELLS, MOZZI_CONTROL_RATE> lfo_pitch1(SIN2048_DATA);
Oscil<SIN2048_NUM_CELLS, MOZZI_CONTROL_RATE> lfo_pitch2(SIN2048_DATA);
Oscil<SIN2048_NUM_CELLS, MOZZI_CONTROL_RATE> lfo_pitch3(SIN2048_DATA);
Oscil<SIN2048_NUM_CELLS, MOZZI_CONTROL_RATE> lfo_pitch4(SIN2048_DATA);

// LFO modulants
Oscil<SIN2048_NUM_CELLS,              MOZZI_CONTROL_RATE> lfo_sin1(SIN2048_DATA);
Oscil<SIN2048_NUM_CELLS,              MOZZI_CONTROL_RATE> lfo_sin2(SIN2048_DATA);
Oscil<SIN2048_NUM_CELLS,              MOZZI_CONTROL_RATE> lfo_sin3(SIN2048_DATA);
Oscil<SIN2048_NUM_CELLS,              MOZZI_CONTROL_RATE> lfo_sin4(SIN2048_DATA);
Oscil<SQUARE_NO_ALIAS_2048_NUM_CELLS, MOZZI_CONTROL_RATE> lfo_sq1(SQUARE_NO_ALIAS_2048_DATA);
Oscil<SQUARE_NO_ALIAS_2048_NUM_CELLS, MOZZI_CONTROL_RATE> lfo_sq2(SQUARE_NO_ALIAS_2048_DATA);
Oscil<SQUARE_NO_ALIAS_2048_NUM_CELLS, MOZZI_CONTROL_RATE> lfo_sq3(SQUARE_NO_ALIAS_2048_DATA);
Oscil<SQUARE_NO_ALIAS_2048_NUM_CELLS, MOZZI_CONTROL_RATE> lfo_sq4(SQUARE_NO_ALIAS_2048_DATA);

uint16_t sh_period[4]  = { 32, 32, 32, 32 };
uint16_t sh_counter[4] = { 0, 0, 0, 0 };
int8_t   sh_val[4]     = { 0, 0, 0, 0 };

#define LFO_PITCH_DEPTH_HZ  5.0f
#define LFO_MOD_DEPTH_HZ   50.0f
#define DETUNE              0.3f

void setLfoVolFreq(uint8_t v, float f) {
  lfo_vol_freq[v] = f;
  sh_period[v] = (uint16_t)constrain(
    (float)MOZZI_CONTROL_RATE / (2.0f * f), 1.0f, 65535.0f);
  switch (v) {
    case 0: lfo_sin1.setFreq(f); lfo_sq1.setFreq(f); break;
    case 1: lfo_sin2.setFreq(f); lfo_sq2.setFreq(f); break;
    case 2: lfo_sin3.setFreq(f); lfo_sq3.setFreq(f); break;
    case 3: lfo_sin4.setFreq(f); lfo_sq4.setFreq(f); break;
  }
}

void tickLfoMod() {
  for (uint8_t v = 0; v < 4; v++) {
    switch (lfo_shape) {
      case LFO_SIN:
        lfo_mod_val[v]=(v==0)?lfo_sin1.next():(v==1)?lfo_sin2.next():(v==2)?lfo_sin3.next():lfo_sin4.next();
        break;
      case LFO_PULSE:
        lfo_mod_val[v]=(v==0)?lfo_sq1.next():(v==1)?lfo_sq2.next():(v==2)?lfo_sq3.next():lfo_sq4.next();
        break;
      case LFO_SH:
        if (++sh_counter[v] >= sh_period[v]) {
          sh_counter[v] = 0;
          sh_val[v] = (int8_t)(random(256)-128);
        }
        lfo_mod_val[v] = sh_val[v];
        break;
    }
  }
}

// ──────────────────────────────────────────────────────────
//  ENVELOPPES AR
// ──────────────────────────────────────────────────────────

uint16_t att_ms[4]    = { 50, 50, 50, 50 };
uint16_t rel_ms[4]    = { 500, 500, 500, 500 };
uint32_t att_rate[4];
uint32_t rel_rate[4];
uint32_t env_level[4] = { 0, 0, 0, 0 };

typedef enum { ENV_IDLE, ENV_ATTACK, ENV_SUSTAIN, ENV_RELEASE } EnvState;
EnvState env_state[4] = { ENV_IDLE, ENV_IDLE, ENV_IDLE, ENV_IDLE };
uint8_t  env_out[4]   = { 255, 255, 255, 255 };

uint16_t potToMs(int pot) {
  return (uint16_t)constrain(
    (int)(5.0f * pow(10.0f, (float)pot/1023.0f*3.0f)), 5, 5000);
}

void computeEnvRates() {
  for (uint8_t v = 0; v < 4; v++) {
    uint32_t at = ((uint32_t)att_ms[v] * MOZZI_CONTROL_RATE) / 1000;
    uint32_t rt = ((uint32_t)rel_ms[v] * MOZZI_CONTROL_RATE) / 1000;
    if (at < 1) at = 1;
    if (rt < 1) rt = 1;
    att_rate[v] = (255UL<<16) / at;
    rel_rate[v] = (255UL<<16) / rt;
  }
}

void trigEnvelopes() {
  for (uint8_t v = 0; v < 4; v++) { env_level[v]=0; env_state[v]=ENV_ATTACK; }
}

void tickEnvelopes(bool trig_held) {
  for (uint8_t v = 0; v < 4; v++) {
    switch (env_state[v]) {
      case ENV_IDLE:    env_out[v]=0; break;
      case ENV_ATTACK:
        env_level[v] += att_rate[v];
        if (env_level[v] >= (255UL<<16)) {
          env_level[v]=(255UL<<16);
          env_state[v]=trig_held?ENV_SUSTAIN:ENV_RELEASE;
        }
        env_out[v]=(uint8_t)(env_level[v]>>16);
        break;
      case ENV_SUSTAIN:
        env_out[v]=255;
        if (!trig_held) env_state[v]=ENV_RELEASE;
        break;
      case ENV_RELEASE:
        if (env_level[v]<=rel_rate[v]) {
          env_level[v]=0; env_out[v]=0;
          env_state[v]=(env_submode==SUBMODE_ENVLOOP)?ENV_ATTACK:ENV_IDLE;
        } else {
          env_level[v]-=rel_rate[v];
          env_out[v]=(uint8_t)(env_level[v]>>16);
        }
        break;
    }
  }
}

// ──────────────────────────────────────────────────────────
//  CONVERSIONS
// ──────────────────────────────────────────────────────────

float potToFreq(int pot) {
  return 20.0f * pow(2.0f, (float)pot/1023.0f*10.0f);
}

float potToLfoFreq(int pot) {
  return 0.05f + (float)pot/1023.0f*19.95f;
}

// ──────────────────────────────────────────────────────────
//  RANDOMISATION
// ──────────────────────────────────────────────────────────

bool btn_rand_prev = false;

void applyRandom() {
  for (uint8_t i = 0; i < NUM_POTS; i++) {
    int r = (int)random(1024);
    pot_applied[current_menu][i] = r;
    pot_ref[i]    = r;
    pot_caught[i] = false;
    switch (current_menu) {
      case 0:
        if (i<4) base_freq[i]       = potToFreq(r);
        else     vol_max[i-4]       = r>>2;
        break;
      case 1:
        if (i<4) setLfoVolFreq(i, potToLfoFreq(r));
        else     lfo_vol_depth[i-4] = r>>2;
        break;
      case 2: {
        uint16_t ms = potToMs(r);
        if (i<4) att_ms[i]=ms; else rel_ms[i-4]=ms;
        computeEnvRates();
        break;
      }
      case 3:
        if (i<4) { filt_cut[i]  =r>>2; computeFilterCoeffs(i); }
        else     { filt_res[i-4]=r>>2; computeFilterCoeffs(i-4); }
        break;
      case 4:
        if (i<4) bc_bits[i]  =r>>2;
        else     bc_rate[i-4]=r>>2;
        break;
    }
  }
}

// ──────────────────────────────────────────────────────────
//  SETUP
// ──────────────────────────────────────────────────────────

void setup() {
  for (uint8_t i=0;i<NUM_MENUS;i++) pinMode(MENU_LEDS[i],OUTPUT);
  pinMode(LED_LFO_TARGET, OUTPUT);
  pinMode(BTN_MENU,    INPUT);
  pinMode(BTN_RAND,    INPUT);
  pinMode(BTN_SUBMENU, INPUT);
  pinMode(BTN_TRIG,    INPUT);
  pinMode(BTN_D,       INPUT);

  updateMenuLeds(0);

  lfo_pitch1.setFreq(0.03f); lfo_pitch2.setFreq(0.05f);
  lfo_pitch3.setFreq(0.07f); lfo_pitch4.setFreq(0.11f);
  for (uint8_t v=0;v<4;v++) setLfoVolFreq(v,1.0f);

  vco1a.setFreq(220.0f); vco1b.setFreq(220.0f+DETUNE);
  vco2a.setFreq(220.0f); vco2b.setFreq(220.0f+DETUNE);
  vco3a.setFreq(220.0f); vco3b.setFreq(220.0f+DETUNE);
  vco4a.setFreq(220.0f); vco4b.setFreq(220.0f+DETUNE);

  for (uint8_t v=0;v<4;v++) computeFilterCoeffs(v);
  computeEnvRates();
  catchupInit();
  initCatchup();
  startMozzi();
}

// ──────────────────────────────────────────────────────────
//  UPDATE CONTROL (64 Hz)
// ──────────────────────────────────────────────────────────

void updateControl() {
  uint32_t now = millis();

  // ── BTN_MENU ─────────────────────────────────────────────
  bool btn_menu_now = (digitalRead(BTN_MENU)==HIGH);
  if (btn_menu_now && !btn_menu_prev && now>=menu_lockout_at) {
    current_menu    = (current_menu+1)%NUM_MENUS;
    menu_lockout_at = now+MENU_LOCKOUT_MS;
    initCatchup();
  }
  btn_menu_prev = btn_menu_now;

  // ── BTN_SUBMENU (D10) ────────────────────────────────────
  bool btn_sub_now = (digitalRead(BTN_SUBMENU)==HIGH);
  if (btn_sub_now && !btn_sub_prev && now>=sub_lockout_at) {
    sub_lockout_at = now+MENU_LOCKOUT_MS;
    if (current_menu==1) {
      lfo_shape = (LfoShape)((lfo_shape+1)%3);
    } else if (current_menu==2) {
      env_submode = (SubMode)((env_submode+1)%3);
      for (uint8_t v=0;v<4;v++) {
        env_level[v]=0; env_state[v]=ENV_IDLE;
        env_out[v]=(env_submode==SUBMODE_HOLD)?255:0;
      }
      if (env_submode==SUBMODE_ENVLOOP) trigEnvelopes();
    }
  }
  btn_sub_prev = btn_sub_now;

  // ── BTN_TRIG (D11) ───────────────────────────────────────
  bool trig_now = (digitalRead(BTN_TRIG)==HIGH);
  if (current_menu==0) {
    // Menu 1 : toggle unison
    if (trig_now && !btn_trig_prev && now>=trig_lockout_at) {
      unison_active   = !unison_active;
      trig_lockout_at = now+MENU_LOCKOUT_MS;
    }
  } else if (current_menu==1) {
    if (trig_now && !btn_trig_prev && now>=trig_lockout_at) {
      lfo_target      = (LfoTarget)((lfo_target+1)%4);
      trig_lockout_at = now+MENU_LOCKOUT_MS;
    }
  } else if (current_menu==2) {
    if (env_submode==SUBMODE_ENV && trig_now && !btn_trig_prev)
      trigEnvelopes();
  }
  btn_trig_prev = trig_now;

  // ── BTN_D12 ──────────────────────────────────────────────
  bool btn_d12_now = (digitalRead(BTN_D)==HIGH);
  if (btn_d12_now && !btn_d12_prev && now>=d12_lockout_at) {
    if (current_menu==0) {
      // Menu 1 : toggle mute global
      mute_global    = !mute_global;
      d12_lockout_at = now+MENU_LOCKOUT_MS;
    } else if (current_menu==4) {
      // Menu 5 : toggle bitcrusher
      bc_active      = !bc_active;
      d12_lockout_at = now+MENU_LOCKOUT_MS;
      if (bc_active)
        for (uint8_t v=0;v<4;v++) { bc_counter[v]=0; bc_held[v]=0; }
    }
  }
  btn_d12_prev = btn_d12_now;

  // ── Enveloppes ───────────────────────────────────────────
  if (env_submode==SUBMODE_HOLD) {
    for (uint8_t v=0;v<4;v++) env_out[v]=255;
  } else {
    tickEnvelopes(trig_now && env_submode==SUBMODE_ENV);
  }

  // ── LEDs ─────────────────────────────────────────────────
  updateMenuLeds(now);

  // ── BTN_RAND (D3) ────────────────────────────────────────
  bool btn_rand_now = (digitalRead(BTN_RAND)==HIGH);
  if (btn_rand_now && !btn_rand_prev) applyRandom();
  btn_rand_prev = btn_rand_now;

  // ── Pots avec catchup ────────────────────────────────────
  switch (current_menu) {
    case 0: {
      int r;
      r=potRead(0); if(r>=0) base_freq[0]=potToFreq(r);
      r=potRead(1); if(r>=0) base_freq[1]=potToFreq(r);
      r=potRead(2); if(r>=0) base_freq[2]=potToFreq(r);
      r=potRead(3); if(r>=0) base_freq[3]=potToFreq(r);
      r=potRead(4); if(r>=0) vol_max[0]=r>>2;
      r=potRead(5); if(r>=0) vol_max[1]=r>>2;
      r=potRead(6); if(r>=0) vol_max[2]=r>>2;
      r=potRead(7); if(r>=0) vol_max[3]=r>>2;
    } break;

    case 1: {
      int r;
      r=potRead(0); if(r>=0) setLfoVolFreq(0,potToLfoFreq(r));
      r=potRead(1); if(r>=0) setLfoVolFreq(1,potToLfoFreq(r));
      r=potRead(2); if(r>=0) setLfoVolFreq(2,potToLfoFreq(r));
      r=potRead(3); if(r>=0) setLfoVolFreq(3,potToLfoFreq(r));
      r=potRead(4); if(r>=0) lfo_vol_depth[0]=r>>2;
      r=potRead(5); if(r>=0) lfo_vol_depth[1]=r>>2;
      r=potRead(6); if(r>=0) lfo_vol_depth[2]=r>>2;
      r=potRead(7); if(r>=0) lfo_vol_depth[3]=r>>2;
    } break;

    case 2: {
      bool ch=false; int r;
      r=potRead(0); if(r>=0){att_ms[0]=potToMs(r);ch=true;}
      r=potRead(1); if(r>=0){att_ms[1]=potToMs(r);ch=true;}
      r=potRead(2); if(r>=0){att_ms[2]=potToMs(r);ch=true;}
      r=potRead(3); if(r>=0){att_ms[3]=potToMs(r);ch=true;}
      r=potRead(4); if(r>=0){rel_ms[0]=potToMs(r);ch=true;}
      r=potRead(5); if(r>=0){rel_ms[1]=potToMs(r);ch=true;}
      r=potRead(6); if(r>=0){rel_ms[2]=potToMs(r);ch=true;}
      r=potRead(7); if(r>=0){rel_ms[3]=potToMs(r);ch=true;}
      if(ch) computeEnvRates();
    } break;

    case 3: { // filtre : cutoff (A0-A3) + resonance (A4-A7)
      int r;
      r=potRead(0); if(r>=0){filt_cut[0]=r>>2; computeFilterCoeffs(0);}
      r=potRead(1); if(r>=0){filt_cut[1]=r>>2; computeFilterCoeffs(1);}
      r=potRead(2); if(r>=0){filt_cut[2]=r>>2; computeFilterCoeffs(2);}
      r=potRead(3); if(r>=0){filt_cut[3]=r>>2; computeFilterCoeffs(3);}
      r=potRead(4); if(r>=0){filt_res[0]=r>>2; computeFilterCoeffs(0);}
      r=potRead(5); if(r>=0){filt_res[1]=r>>2; computeFilterCoeffs(1);}
      r=potRead(6); if(r>=0){filt_res[2]=r>>2; computeFilterCoeffs(2);}
      r=potRead(7); if(r>=0){filt_res[3]=r>>2; computeFilterCoeffs(3);}
    } break;

    case 4: {
      int r;
      r=potRead(0); if(r>=0) bc_bits[0]=r>>2;
      r=potRead(1); if(r>=0) bc_bits[1]=r>>2;
      r=potRead(2); if(r>=0) bc_bits[2]=r>>2;
      r=potRead(3); if(r>=0) bc_bits[3]=r>>2;
      r=potRead(4); if(r>=0) bc_rate[0]=r>>2;
      r=potRead(5); if(r>=0) bc_rate[1]=r>>2;
      r=potRead(6); if(r>=0) bc_rate[2]=r>>2;
      r=potRead(7); if(r>=0) bc_rate[3]=r>>2;
    } break;
  }

  // ── LFO pitch ────────────────────────────────────────────
  lfo_pitch_val[0]=lfo_pitch1.next(); lfo_pitch_val[1]=lfo_pitch2.next();
  lfo_pitch_val[2]=lfo_pitch3.next(); lfo_pitch_val[3]=lfo_pitch4.next();

  // ── LFO modulant ─────────────────────────────────────────
  tickLfoMod();

  // ── Application cible LFO + vol_eff ──────────────────────
  for (uint8_t v=0;v<4;v++) {
    uint8_t depth = lfo_vol_depth[v];
    // Valeur LFO normalisee 0..255
    uint8_t uni = (uint8_t)((int16_t)lfo_mod_val[v]+128);

    switch (lfo_target) {
      case LFO_TARGET_VOL:
        filt_f_eff[v]=filt_f[v]; // pas de modulation filtre
        if (depth==0) {
          vol_eff[v]=(uint8_t)((uint16_t)vol_max[v]*env_out[v]>>8);
        } else {
          uint8_t mod=(uint8_t)((uint16_t)uni*depth>>8);
          uint8_t fac=(255-depth)+mod;
          uint8_t lvo=(uint8_t)((uint16_t)vol_max[v]*fac>>8);
          vol_eff[v]=(uint8_t)((uint16_t)lvo*env_out[v]>>8);
        }
        break;

      case LFO_TARGET_PITCH:
        filt_f_eff[v]=filt_f[v];
        vol_eff[v]=(uint8_t)((uint16_t)vol_max[v]*env_out[v]>>8);
        break;

      case LFO_TARGET_FILTER:
        vol_eff[v]=(uint8_t)((uint16_t)vol_max[v]*env_out[v]>>8);
        if (depth>0) {
          int32_t mod=((int32_t)(uni-128)*depth*16)>>8;
          int32_t f_mod=(int32_t)filt_f[v]+mod;
          if (f_mod<1)    f_mod=1;
          if (f_mod>4095) f_mod=4095;
          filt_f_eff[v]=(int16_t)f_mod;
        } else {
          filt_f_eff[v]=filt_f[v];
        }
        break;

      case LFO_TARGET_CRUSH:
        filt_f_eff[v]=filt_f[v];
        vol_eff[v]=(uint8_t)((uint16_t)vol_max[v]*env_out[v]>>8);
        if (bc_active && depth>0) {
          uint8_t mod=(uint8_t)((uint16_t)uni*depth>>8);
          bc_rate[v]=mod;
        }
        break;
    }
  }

  // ── Freq VCO ─────────────────────────────────────────────
  static float last_f[4]={0,0,0,0};
  {
    float p0=((float)lfo_pitch_val[0]*LFO_PITCH_DEPTH_HZ)/128.0f;
    float p1=((float)lfo_pitch_val[1]*LFO_PITCH_DEPTH_HZ)/128.0f;
    float p2=((float)lfo_pitch_val[2]*LFO_PITCH_DEPTH_HZ)/128.0f;
    float p3=((float)lfo_pitch_val[3]*LFO_PITCH_DEPTH_HZ)/128.0f;

    float m0=0,m1=0,m2=0,m3=0;
    if (lfo_target==LFO_TARGET_PITCH) {
      float sc=LFO_MOD_DEPTH_HZ/128.0f;
      m0=(float)lfo_mod_val[0]*((float)lfo_vol_depth[0]/255.0f)*sc;
      m1=(float)lfo_mod_val[1]*((float)lfo_vol_depth[1]/255.0f)*sc;
      m2=(float)lfo_mod_val[2]*((float)lfo_vol_depth[2]/255.0f)*sc;
      m3=(float)lfo_mod_val[3]*((float)lfo_vol_depth[3]/255.0f)*sc;
    }

    float f0=base_freq[0]+p0+m0; if(f0<1)f0=1;
    // Unison : VCO2/3/4 prennent la freq de VCO1 (+ leur LFO propre)
    float f1=unison_active ? base_freq[0]+p1+m1 : base_freq[1]+p1+m1; if(f1<1)f1=1;
    float f2=unison_active ? base_freq[0]+p2+m2 : base_freq[2]+p2+m2; if(f2<1)f2=1;
    float f3=unison_active ? base_freq[0]+p3+m3 : base_freq[3]+p3+m3; if(f3<1)f3=1;

    if(f0<last_f[0]-0.1f||f0>last_f[0]+0.1f){vco1a.setFreq(f0);vco1b.setFreq(f0+DETUNE);last_f[0]=f0;}
    if(f1<last_f[1]-0.1f||f1>last_f[1]+0.1f){vco2a.setFreq(f1);vco2b.setFreq(f1+DETUNE);last_f[1]=f1;}
    if(f2<last_f[2]-0.1f||f2>last_f[2]+0.1f){vco3a.setFreq(f2);vco3b.setFreq(f2+DETUNE);last_f[2]=f2;}
    if(f3<last_f[3]-0.1f||f3>last_f[3]+0.1f){vco4a.setFreq(f3);vco4b.setFreq(f3+DETUNE);last_f[3]=f3;}
  }
}

// ──────────────────────────────────────────────────────────
//  BITCRUSHER
// ──────────────────────────────────────────────────────────

int32_t applyBitcrush(int32_t s, uint8_t bits_amt, uint8_t rate_amt, uint8_t v) {
  if (rate_amt>0) {
    uint8_t hold=1+(rate_amt>>3);
    if (++bc_counter[v]>=hold) {bc_counter[v]=0;bc_held[v]=s;}
    s=bc_held[v];
  }
  if (bits_amt>0) {
    uint8_t crush=1+(bits_amt>>5);
    if (crush>7) crush=7;
    int8_t s8=(int8_t)constrain(s>>1,-128,127);
    int8_t mask=(int8_t)(0xFF<<crush);
    s=(int32_t)(s8&mask)<<1;
  }
  return s;
}

// ──────────────────────────────────────────────────────────
//  UPDATE AUDIO
// ──────────────────────────────────────────────────────────

AudioOutput updateAudio() {
  // Mute global (D12 menu 1)
  if (mute_global) return MonoOutput::fromNBit(8, 128);

  int32_t s1=0,s2=0,s3=0,s4=0;

  // Lecture oscillateurs seulement si VCO audible
  if (vol_eff[0]) s1=(int32_t)vco1a.next()+vco1b.next();
  if (vol_eff[1]) s2=(int32_t)vco2a.next()+vco2b.next();
  if (vol_eff[2]) s3=(int32_t)vco3a.next()+vco3b.next();
  if (vol_eff[3]) s4=(int32_t)vco4a.next()+vco4b.next();

  // Filtre SVF (bypass si VCO silencieux)
  if (vol_eff[0]) s1=applyFilter(s1,0);
  if (vol_eff[1]) s2=applyFilter(s2,1);
  if (vol_eff[2]) s3=applyFilter(s3,2);
  if (vol_eff[3]) s4=applyFilter(s4,3);

  // Bitcrusher
  if (bc_active) {
    if (vol_eff[0]) s1=applyBitcrush(s1,bc_bits[0],bc_rate[0],0);
    if (vol_eff[1]) s2=applyBitcrush(s2,bc_bits[1],bc_rate[1],1);
    if (vol_eff[2]) s3=applyBitcrush(s3,bc_bits[2],bc_rate[2],2);
    if (vol_eff[3]) s4=applyBitcrush(s4,bc_bits[3],bc_rate[3],3);
  }

  // Volume
  s1=(s1*(int32_t)vol_eff[0])>>8;
  s2=(s2*(int32_t)vol_eff[1])>>8;
  s3=(s3*(int32_t)vol_eff[2])>>8;
  s4=(s4*(int32_t)vol_eff[3])>>8;

  int32_t mix=(s1+s2+s3+s4)>>3;
  if (mix> 127) mix= 127;
  if (mix<-128) mix=-128;
  return MonoOutput::fromNBit(8,(int)(mix+128));
}

// ──────────────────────────────────────────────────────────
void loop() {
  audioHook();
}
