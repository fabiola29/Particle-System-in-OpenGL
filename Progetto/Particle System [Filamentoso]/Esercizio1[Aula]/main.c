/*
    particle_system_glass.c
    di Salomone Fabiola
    (riferimento : Approximate and Probabilistic Algorithms
    for Shading and Rendering Structured Particle Systems di
    William T. Reeves)
*/

/*  ************************************************************************
    DESCRIZIONE

    In questo file è presentata una possibile implementazione di un sistema di particelle
    ottimizzato per la simulazione e il rendering di oggetti costituiti da molteplici filamenti,
    con particolare attenzione alla resa realistica dell’erba.
        
*/

/*   ************************************************************************ */
// LIBRERIE
#define GL_SILENCE_DEPRECATION
#include <GLFW/glfw3.h>
#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <stdbool.h>
#include <string.h>

/*    ************************************************************************ */
// ENUMERAZIONI, COSTANTI E VARIABILI GLOBALI
#define NUM_CLUMPS 3000
#define MAX_BLADES_PER_CLUMP 50
#define WIDTH 800
#define HEIGHT 600
#define SHADOW_MAP_RES 512
#define WIND_MAP_RES 64
#define MAX_BENDING_FUNCTIONS 5
#define MAX_ACTIVE_FUNCTIONS 5
#define MAX_BUCKETS 200
#define MAX_BLADES_PER_BUCKET (NUM_CLUMPS * MAX_BLADES_PER_CLUMP / MAX_BUCKETS)

typedef struct {
    float start_time;   // Tempo d'inizio della funzione di bending
    float amplitude;    // Ampiezza dell'oscillazione
    float frequency;    // Frequenza dell'oscillazione
    float decay;        // Velocità di decadimento dell'effetto
} BendingFunction;

typedef struct {
    BendingFunction functions[MAX_ACTIVE_FUNCTIONS];  // Array di funzioni di bending attive
    int count;                                        // Numero di funzioni attive
} BendingFunctionSet;

typedef struct {
    float x, y, z;           // Posizione 3D della lama d'erba
    float height;            // Altezza della lama
    float curvature;         // Curvatura della lama
    float thickness;         // Spessore della lama
    float r, g, b;           // Colore base della lama (RGB)
    float shadow_factor;     // Fattore d'ombra sulla lama
    int segments;            // Numero di segmenti per la geometria della lama
    float wind_offset;       // Offset per variazioni di vento
    float depth_in_clump;    // Profondità della lama nel ciuffo (per ombra/attenuazione)
    bool has_flower;         // Indica se la lama ha un fiore
    float flower_r, flower_g, flower_b; // Colore del fiore, se presente
    BendingFunctionSet bends;            // Set di funzioni di bending attive sulla lama
} GrassBlade;

typedef struct {
    float x, z;              // Posizione del ciuffo nel piano XZ
    float orientation;       // Orientamento del ciuffo
    int count;               // Numero di lame nel ciuffo
    GrassBlade blades[MAX_BLADES_PER_CLUMP];  // Array di lame d'erba nel ciuffo
} GrassClump;

typedef struct {
    GrassBlade* blades[MAX_BLADES_PER_BUCKET];  // Puntatori a lame d'erba contenute nel bucket
    int count;                                  // Numero di lame nel bucket
} BladeBucket;

// POSIZIONE E DIREZIONE
float camX = 0.0f, camZ = 8.0f;        // Posizione della telecamera nel piano XZ
float light_dir[3] = { -0.4f, 1.0f, -0.5f };  // Direzione della luce (vettore)

//MAPPE E ARRAY GLOBALI
unsigned char shadow_mask[SHADOW_MAP_RES][SHADOW_MAP_RES];      // Mappa dell'ombra statica (es. ombre proiettate)
unsigned char terrain_map[SHADOW_MAP_RES][SHADOW_MAP_RES];     // Mappa del terreno (es. tipo o altezza terreno)
float wind_map[WIND_MAP_RES][WIND_MAP_RES];                    // Mappa del vento per simulazioni dinamiche

GrassClump clumps[NUM_CLUMPS];                                 // Array di ciuffi d'erba nell'area
GrassBlade* visible_blades[NUM_CLUMPS * MAX_BLADES_PER_CLUMP]; // Array di puntatori a lame visibili (per rendering)
int visible_count = 0;                                         // Numero di lame visibili

BladeBucket buckets[MAX_BUCKETS];                              // Suddivisione in bucket per gestione ottimizzata

unsigned char vegetation_map[SHADOW_MAP_RES][SHADOW_MAP_RES]; // Mappa dei tipi di vegetazione

float dynamic_shadow_map[SHADOW_MAP_RES][SHADOW_MAP_RES];     // Mappa dinamica per ombre variabili nel tempo

unsigned char shadow_mask[SHADOW_MAP_RES][SHADOW_MAP_RES];

unsigned char terrain_map[SHADOW_MAP_RES][SHADOW_MAP_RES];

float wind_map[WIND_MAP_RES][WIND_MAP_RES];


/*    ************************************************************************ */
// FUNZIONI ACCESSORIE

/**
 * Calcola la distanza euclidea nel piano XZ tra una lama d'erba e la posizione della camera.
 *
 * Parametri di input:
 *   - blade: puntatore a una struttura GrassBlade, che rappresenta una singola lama d'erba.
 *            La funzione usa le coordinate x e z della lama per il calcolo.
 *
 * Valore restituito:
 *   - float: la distanza tra la lama e la camera nel piano XZ.
 */
float camera_depth(GrassBlade* blade) {
    // Calcola la differenza lungo l'asse X tra la lama e la camera
    float dx = blade->x - camX;

    // Calcola la differenza lungo l'asse Z tra la lama e la camera
    float dz = blade->z - camZ;

    // Calcola e restituisce la distanza euclidea nel piano XZ usando il teorema di Pitagora
    return sqrtf(dx * dx + dz * dz);
}


/**
 * generate_vegetation_map - Genera una mappa 2D di tipi di vegetazione casuale.
 *
 * Non prende parametri in input e modifica la variabile globale `vegetation_map`.
 *
 * La funzione assegna a ogni cella della mappa un tipo di vegetazione con percentuali predefinite:
 *   - 10%: zona arida (nessuna vegetazione, codice 0)
 *   - 20%: erba secca o gialla (codice 1)
 *   - 40%: erba normale (codice 2)
 *   - 30%: zona con fiori (codice 3)
 *
 * Non restituisce nulla.
 */
void generate_vegetation_map() {
    // Cicla su tutte le righe della mappa
    for (int i = 0; i < SHADOW_MAP_RES; i++) {
        // Cicla su tutte le colonne della mappa
        for (int j = 0; j < SHADOW_MAP_RES; j++) {
            int r = rand() % 100;  // Genera un numero casuale tra 0 e 99

            // Assegna il tipo di vegetazione in base al valore casuale r, secondo percentuali:
            if (r < 10)
                vegetation_map[i][j] = 0;   // 10% di probabilità: zona arida (nessuna vegetazione)
            else if (r < 30)
                vegetation_map[i][j] = 1;   // 20% di probabilità: erba secca o gialla
            else if (r < 70)
                vegetation_map[i][j] = 2;   // 40% di probabilità: erba normale
            else
                vegetation_map[i][j] = 3;   // 30% di probabilità: zona con fiori
        }
    }
}

/**
 * vegetation_type - Restituisce il tipo di vegetazione per una posizione (x, z).
 *
 * Parametri:
 *   float x - coordinata orizzontale nel range approssimativo [-10, 10]
 *   float z - coordinata verticale nel range approssimativo [-10, 10]
 *
 * Valore restituito:
 *   int - tipo di vegetazione nella posizione data (codici come in generate_vegetation_map)
 *         0 = nessuna vegetazione (zona arida)
 *         1 = erba secca/gialla
 *         2 = erba normale
 *         3 = zona con fiori
 *         Se la posizione è fuori dai limiti della mappa, restituisce 0.
 */
int vegetation_type(float x, float z) {
    // Calcola l'indice orizzontale (sx) nella mappa della vegetazione
    // Mappa il range (-10..10) in (0..20), normalizza e scala in base alla risoluzione SHADOW_MAP_RES
    int sx = (int)((x + 10.0f) / 20.0f * SHADOW_MAP_RES);

    // Calcola l'indice verticale (sz) nella mappa della vegetazione con lo stesso metodo
    int sz = (int)((z + 10.0f) / 20.0f * SHADOW_MAP_RES);

    // Se gli indici sono fuori dai limiti della mappa, restituisce 0 (nessuna vegetazione)
    if (sx < 0 || sx >= SHADOW_MAP_RES || sz < 0 || sz >= SHADOW_MAP_RES)
        return 0;

    // Restituisce il valore della mappa della vegetazione alla posizione calcolata
    return vegetation_map[sz][sx];
}

/**
 * randf_range - Genera un numero float casuale nell'intervallo [min, max].
 *
 * Parametri:
 *   float min - valore minimo dell'intervallo
 *   float max - valore massimo dell'intervallo
 *
 * Valore restituito:
 *   float - valore casuale compreso tra min e max
 */
float randf_range(float min, float max) {
    // Genera un float casuale normalizzato tra 0 e 1
    // (float)rand() / RAND_MAX produce un valore nell'intervallo [0, 1]
    float normalized = (float)rand() / RAND_MAX;

    // Scala e trasla il valore normalizzato per ottenere il range desiderato
    return min + normalized * (max - min);
}

/**
 * rand_gaussian - Genera un numero casuale con distribuzione gaussiana (normale).
 *
 * Parametri:
 *   float mean - media della distribuzione
 *   float stddev - deviazione standard della distribuzione
 *
 * Valore restituito:
 *   float - valore casuale con distribuzione normale di media 'mean' e deviazione 'stddev'
 */
float rand_gaussian(float mean, float stddev) {
    // Genera due numeri casuali uniformi nell'intervallo (0,1], evitando zero
    float u1 = ((float)rand() + 1.0f) / ((float)RAND_MAX + 1.0f);
    float u2 = ((float)rand() + 1.0f) / ((float)RAND_MAX + 1.0f);

    // Applica la trasformazione di Box-Muller per ottenere una variabile casuale normale standard
    // z0 ha media 0 e deviazione standard 1
    float z0 = sqrtf(-2.0f * logf(u1)) * cosf(2 * M_PI * u2);

    // Scala e trasla z0 per ottenere la distribuzione con media 'mean' e deviazione 'stddev'
    return mean + stddev * z0;
}

/**
 * normalize - Normalizza un vettore 3D in place.
 *
 * Parametri:
 *   float* v - puntatore a un array di 3 float rappresentante il vettore (x, y, z)
 *
 * Output:
 *   Il vettore 'v' viene modificato in modo che la sua lunghezza sia 1 (vettore unitario),
 *   se la lunghezza originale era maggiore di zero.
 */
void normalize(float* v) {
    // Calcola la lunghezza (modulo) del vettore usando la radice quadrata della somma dei quadrati
    float len = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);

    // Se la lunghezza è positiva, normalizza ogni componente dividendo per la lunghezza
    if (len > 0.0f) {
        v[0] /= len;  // x diviso per la lunghezza
        v[1] /= len;  // y diviso per la lunghezza
        v[2] /= len;  // z diviso per la lunghezza
    }
    // Se la lunghezza è zero, non fa nulla per evitare divisione per zero
}

/**
 * sample_shadow_mask - Campiona il valore della maschera d'ombra alla posizione (x, z).
 *
 * Parametri:
 *   float x - coordinata orizzontale nello spazio (range atteso [-10, 10])
 *   float z - coordinata verticale nello spazio (range atteso [-10, 10])
 *
 * Valore restituito:
 *   unsigned char - valore di ombra nella mappa alla posizione calcolata,
 *                   oppure 255 se la posizione è fuori dalla mappa.
 */
unsigned char sample_shadow_mask(float x, float z) {
    // Calcola l'indice orizzontale (sx) corrispondente alla posizione x nella mappa della maschera d'ombra
    // Mappa il range (-10..10) in (0..20), quindi lo normalizza e scala rispetto alla risoluzione SHADOW_MAP_RES
    int sx = (int)((x + 10.0f) / 20.0f * SHADOW_MAP_RES);

    // Calcola l'indice verticale (sz) corrispondente alla posizione z nella mappa
    int sz = (int)((z + 10.0f) / 20.0f * SHADOW_MAP_RES);

    // Se gli indici sono fuori dai limiti della mappa, restituisce 255 (valore speciale)
    if (sx < 0 || sx >= SHADOW_MAP_RES || sz < 0 || sz >= SHADOW_MAP_RES)
        return 255;

    // Restituisce il valore della maschera d'ombra alla posizione calcolata
    return shadow_mask[sz][sx];
}

/**
 * sample_terrain_map - Campiona il tipo di terreno alla posizione (x, z).
 *
 * Parametri:
 *   float x - coordinata orizzontale nello spazio (range atteso [-10, 10])
 *   float z - coordinata verticale nello spazio (range atteso [-10, 10])
 *
 * Valore restituito:
 *   unsigned char - valore del tipo terreno (ad esempio 0, 64, 128, 192) alla posizione,
 *                   oppure 255 se la posizione è fuori dalla mappa.
 */
unsigned char sample_terrain_map(float x, float z) {
    // Calcola l'indice orizzontale (sx) corrispondente alla posizione x
    // - sposta x di +10 per trasformare il range (-10..10) in (0..20)
    // - normalizza dividendo per 20 per ottenere un valore da 0 a 1
    // - moltiplica per la risoluzione della mappa SHADOW_MAP_RES per ottenere l'indice pixel
    int sx = (int)((x + 10.0f) / 20.0f * SHADOW_MAP_RES);

    // Calcola l'indice verticale (sz) corrispondente alla posizione z con la stessa logica
    int sz = (int)((z + 10.0f) / 20.0f * SHADOW_MAP_RES);

    // Controlla se gli indici calcolati sono fuori dal range della mappa
    if (sx < 0 || sx >= SHADOW_MAP_RES || sz < 0 || sz >= SHADOW_MAP_RES)
        return 255;  // Se fuori range, restituisce 255 (valore speciale, ad esempio "nessun dato")

    // Se dentro range, restituisce il valore della mappa del terreno alla posizione calcolata
    return terrain_map[sz][sx];
}

/**
 * sample_wind_map - Campiona il valore del vento in una posizione (x, z) e in un dato tempo
 *                   simulando un movimento ciclico del vento sulla mappa del vento.
 *
 * Parametri:
 *   float x    - coordinata orizzontale nello spazio (range atteso [-10, 10])
 *   float z    - coordinata verticale nello spazio (range atteso [-10, 10])
 *   float time - tempo corrente, usato per far "muovere" la mappa del vento nel tempo
 *
 * Valore restituito:
 *   float - valore del vento campionato dalla mappa in quella posizione e tempo,
 *           tipicamente in [0.0, 1.0)
 */
float sample_wind_map(float x, float z, float time) {
    // Calcola l'indice orizzontale (wx) nella mappa del vento:
    // - sposta x di +10 per mappare il range (-10..10) in (0..20)
    // - normalizza in [0,1] dividendo per 20
    // - scala rispetto alla risoluzione della mappa WIND_MAP_RES
    // - aggiunge un offset basato sul tempo per simulare il movimento del vento nel tempo (velocità 0.5)
    // - applica modulo WIND_MAP_RES per fare wrap-around ciclico sugli indici della mappa
    int wx = ((int)((x + 10.0f) / 20.0f * WIND_MAP_RES) + (int)(time * 0.5f)) % WIND_MAP_RES;

    // Calcola l'indice verticale (wz) nella mappa del vento con un offset temporale differente (velocità 0.6)
    int wz = ((int)((z + 10.0f) / 20.0f * WIND_MAP_RES) + (int)(time * 0.6f)) % WIND_MAP_RES;

    // Restituisce il valore della mappa del vento alla posizione calcolata
    return wind_map[wz][wx];
}

/**
 * add_bending_function - Aggiunge una nuova funzione di piegatura (bending) a una lama d'erba,
 *                        per simulare la risposta al vento.
 *
 * Parametri:
 *   GrassBlade* blade      - puntatore alla lama d’erba a cui aggiungere la funzione
 *   float time            - tempo corrente, usato per marcare l'inizio della funzione
 *   float wind_intensity  - intensità del vento, usata per calcolare l’ampiezza della piegatura
 *
 * Valore restituito:
 *   void - modifica direttamente la struttura blade aggiornandone il set di bending
 */
void add_bending_function(GrassBlade* blade, float time, float wind_intensity) {
    // Controlla se il numero massimo di funzioni attive è stato raggiunto
    if (blade->bends.count >= MAX_ACTIVE_FUNCTIONS) {
        // Se sì, sposta tutte le funzioni di bending un indice indietro,
        // eliminando così la più vecchia (indice 0) per fare spazio a quella nuova
        memmove(&blade->bends.functions[0], &blade->bends.functions[1], sizeof(BendingFunction) * (MAX_ACTIVE_FUNCTIONS - 1));
        blade->bends.count--;  // Aggiorna il contatore delle funzioni attive
    }

    // Prendi il puntatore alla nuova funzione da aggiungere, nella posizione successiva libera
    BendingFunction* b = &blade->bends.functions[blade->bends.count++];

    // Inizializza i parametri della nuova funzione di piegatura
    b->start_time = time;                          // tempo di inizio della funzione
    b->amplitude = 0.1f * wind_intensity;         // ampiezza proporzionale all'intensità del vento
    b->frequency = 0.5f + 0.5f * randf_range(0, 1); // frequenza casuale tra 0.5 e 1.0 Hz
    b->decay = 0.8f;                              // tasso di decadimento esponenziale (velocità di dissipazione)
}

/**
 * apply_bending - Calcola la piegatura totale di una lama d’erba
 *                 combinando più funzioni di bending (oscillazioni temporali con decadimento)
 *                 modulata dall’altezza relativa della lama.
 *
 * Parametri:
 *   BendingFunctionSet* set     - puntatore a un insieme di funzioni di bending da applicare
 *   float t                    - tempo corrente, usato per calcolare l’oscillazione e il decadimento
 *   float blade_height_ratio   - rapporto dell’altezza relativa della lama (0 = base, 1 = punta),
 *                               usato per modulare l’intensità della piegatura sulla lama
 *
 * Valore restituito:
 *   float - valore totale della piegatura calcolata come somma degli effetti di tutte le funzioni di bending attive
 */
float apply_bending(BendingFunctionSet* set, float t, float blade_height_ratio) {
    float total = 0.0f;  // Inizializza il valore totale della piegatura a zero

    // Cicla su tutte le funzioni di bending contenute nel set
    for (int i = 0; i < set->count; i++) {
        BendingFunction* b = &set->functions[i];  // Ottieni puntatore alla funzione corrente

        float dt = t - b->start_time;  // Calcola il tempo trascorso dall'inizio della funzione

        if (dt < 0)
            continue;  // Se la funzione non è ancora iniziata, salta al prossimo ciclo

        // Calcola il contributo della funzione corrente:
        // - ampiezza moltiplicata per
        // - un decadimento esponenziale nel tempo (simula il dissiparsi dell'effetto)
        // - un'oscillazione sinusoidale basata sulla frequenza e sul tempo trascorso
        // - moltiplicato per il rapporto di altezza della lama, per modulare l'intensità in base alla posizione sulla lama
        total += b->amplitude * expf(-b->decay * dt) * sinf(2 * M_PI * b->frequency * dt) * blade_height_ratio;
    }

    return total;  // Restituisce la somma di tutti gli effetti di piegatura
}

/**
 * compute_colored_shading - Calcola il colore finale di una lama d’erba
 *                           tenendo conto dell’illuminazione ambientale,
 *                           diretta, dell’auto-ombra e di un effetto speculare stocastico
 *                           che simula riflessi intermittenti.
 *
 * Parametri:
 *   GrassBlade* blade - puntatore alla struttura della lama d’erba, contiene colore base e profondità nel ciuffo
 *   float dot         - prodotto scalare tra la normale della lama e la direzione della luce (quanto la lama è rivolta verso la luce)
 *   float attenuation - fattore di attenuazione della luce (non usato direttamente qui, ma disponibile per estensioni)
 *   bool in_light     - true se la lama è illuminata direttamente dalla luce solare, false se è in ombra
 *   float* r, g, b    - puntatori a variabili dove verrà scritto il colore calcolato (componenti rosso, verde e blu)
 *   float time        - tempo corrente (usato per calcolare la fase giorno/notte e animazioni)
 *
 * Valore restituito:
 *   void - il colore risultante viene scritto tramite i puntatori r, g, b
 */
void compute_colored_shading(
    GrassBlade* blade,
    float dot,
    float attenuation,
    bool in_light,
    float* r, float* g, float* b,
    float time
) {
    // Calcola la fase del giorno (0.0 a 1.0) con ciclo di 30 secondi
    float day_cycle = fmodf(time, 30.0f) / 30.0f;
    float sun_angle = day_cycle * 2.0f * M_PI;      // angolo solare nel ciclo
    float day_phase = sinf(sun_angle) * 0.5f + 0.5f; // varia da 0 (notte) a 1 (mezzogiorno)

    // Luce ambientale: più fredda/blu di notte, più calda/gialla di giorno
    float ambient_light[3] = {
        0.1f + 0.2f * day_phase,            // rosso aumenta col sole
        0.2f + 0.3f * day_phase,            // verde aumenta col sole
        0.4f + 0.6f * (1.0f - day_phase)   // blu diminuisce col sole
    };

    // Luce diffusa: più calda e intensa a mezzogiorno
    float diffuse_light[3] = {
        0.6f + 0.4f * powf(day_phase, 1.5f),
        0.5f + 0.3f * powf(day_phase, 1.5f),
        0.3f + 0.1f * powf(day_phase, 1.5f)
    };

    // Colore della luce solare diretta (giallo caldo)
    float sun_color[3] = { 1.0f, 0.9f, 0.6f };

    // Colore ambientale base della lama moltiplicato per luce ambientale
    float ambient[3] = {
        blade->r * ambient_light[0],
        blade->g * ambient_light[1],
        blade->b * ambient_light[2]
    };

    // Colore diffuso base moltiplicato per luce diffusa e colore solare
    float diffuse[3] = {
        blade->r * diffuse_light[0] * sun_color[0],
        blade->g * diffuse_light[1] * sun_color[1],
        blade->b * diffuse_light[2] * sun_color[2]
    };

    // Auto-ombra: attenuazione più forte se la lama è più profonda nel ciuffo
    float ambient_factor = expf(-2.0f * blade->depth_in_clump);
    float diffuse_factor = expf(-5.0f * blade->depth_in_clump);

    // Applica attenuazione all'illuminazione ambientale
    *r = ambient[0] * ambient_factor;
    *g = ambient[1] * ambient_factor;
    *b = ambient[2] * ambient_factor;

    if (in_light) {
        // Speculare stocastico: specularità possibile se dot e profondità nel ciuffo sono favorevoli
        bool has_specular = (randf_range(0.0f, 1.0f) < dot * (1.0f - blade->depth_in_clump));
        float specular = has_specular ? powf(dot, 6.0f) : 0.0f;

        // Aggiunge illuminazione diffusa attenuata e componente speculare
        *r += diffuse[0] * diffuse_factor + specular * 1.0f;
        *g += diffuse[1] * diffuse_factor + specular * 0.9f;
        *b += diffuse[2] * diffuse_factor + specular * 0.5f;
    }
}

/**
 * generate_shadow_mask
 * --------------------
 * Inizializza la mappa di ombra (shadow_mask) con un pattern a scacchiera.
 *
 * Descrizione:
 *  - La mappa è suddivisa in blocchi 32x32 celle.
 *  - Per ogni cella, si calcola la somma degli indici di blocco (i/32 + j/32).
 *  - Se la somma è pari, la cella riceve valore 255 (piena ombra).
 *  - Se la somma è dispari, la cella riceve valore 100 (ombra più chiara).
 *
 * Questo crea un pattern a scacchiera con zone di ombra alternate.
 */
void generate_shadow_mask() {
    // Scorre tutte le celle della mappa di ombra (dimensione SHADOW_MAP_RES x SHADOW_MAP_RES)
    for (int i = 0; i < SHADOW_MAP_RES; i++)
        for (int j = 0; j < SHADOW_MAP_RES; j++)
            // Divide le coordinate per 32 per creare blocchi 32x32
            // Somma i blocchi e prende il modulo 2 per alternare i pattern a scacchiera
            // Se la somma è pari, assegna valore massimo di ombra (255, piena ombra)
            // Altrimenti assegna un valore minore (100, meno ombra)
            shadow_mask[i][j] = ((i / 32 + j / 32) % 2 == 0) ? 255 : 100;
}

/**
 * generate_terrain_map
 * --------------------
 * Inizializza la mappa del terreno (terrain_map) assegnando tipi di terreno casuali
 * con una distribuzione probabilistica predefinita.
 *
 * Descrizione:
 *  - Per ogni cella, genera un numero casuale da 0 a 99.
 *  - Assegna il tipo di terreno in base al valore del numero:
 *      10% spoglia (0),
 *      30% erba rada (64),
 *      40% erba normale (128),
 *      20% erba alta/scura (192).
 */
void generate_terrain_map() {
    // Scorre tutte le celle della mappa del terreno (SHADOW_MAP_RES x SHADOW_MAP_RES)
    for (int i = 0; i < SHADOW_MAP_RES; i++) {
        for (int j = 0; j < SHADOW_MAP_RES; j++) {
            // Genera un numero casuale da 0 a 99
            int r = rand() % 100;

            // Assegna il tipo di terreno in base al valore casuale, secondo distribuzione probabilistica:
            // 10% chance zona spoglia (valore 0)
            if (r < 10)
                terrain_map[i][j] = 0;
            // 30% chance erba rada (valore 64)
            else if (r < 40)
                terrain_map[i][j] = 64;
            // 40% chance erba normale (valore 128)
            else if (r < 80)
                terrain_map[i][j] = 128;
            // 20% chance erba alta/scura (valore 192)
            else
                terrain_map[i][j] = 192;
        }
    }
}

/**
 * terrain_type
 * ------------
 * Restituisce il tipo di terreno in una posizione (x, z) specifica.
 *
 * Parametri:
 *  - float x: coordinata X nel range [-10,10].
 *  - float z: coordinata Z nel range [-10,10].
 *
 * Restituisce:
 *  - int: valore intero che rappresenta il tipo di terreno dalla mappa terrain_map.
 *         Se la posizione è fuori dai limiti, ritorna 0 (tipo terreno default).
 *
 * Descrizione:
 *  Converte le coordinate spaziali in indici della mappa del terreno e ritorna
 *  il valore corrispondente. Utile per ottenere caratteristiche locali del terreno.
 */
int terrain_type(float x, float z) {
    // Converte la coordinata x da [-10,10] a un indice di colonna nella mappa (0 .. SHADOW_MAP_RES-1)
    int sx = (int)((x + 10.0f) / 20.0f * SHADOW_MAP_RES);

    // Converte la coordinata z da [-10,10] a un indice di riga nella mappa (0 .. SHADOW_MAP_RES-1)
    int sz = (int)((z + 10.0f) / 20.0f * SHADOW_MAP_RES);

    // Controlla che gli indici siano validi, altrimenti ritorna 0 (es. tipo terreno default)
    if (sx < 0 || sx >= SHADOW_MAP_RES || sz < 0 || sz >= SHADOW_MAP_RES)
        return 0;

    // Ritorna il valore memorizzato nella mappa del terreno alla posizione calcolata
    return terrain_map[sz][sx];
}

/**
 * generate_wind_map
 * -----------------
 * Inizializza la mappa del vento assegnando valori casuali a ogni cella.
 *
 * Parametri:
 *  - Nessuno
 *
 * Restituisce:
 *  - void
 *
 * Descrizione:
 *  Per ogni cella nella mappa 2D wind_map di dimensione WIND_MAP_RES x WIND_MAP_RES,
 *  assegna un valore casuale compreso tra 0.0 (incluso) e 1.0 (escluso).
 *  Questa mappa può essere usata per simulare variazioni locali di intensità del vento.
 */
void generate_wind_map() {
    // Cicla su tutte le posizioni della mappa del vento (risoluzione WIND_MAP_RES x WIND_MAP_RES)
    for (int i = 0; i < WIND_MAP_RES; i++)
        for (int j = 0; j < WIND_MAP_RES; j++)
            // Assegna a ogni cella un valore casuale in [0.0, 1.0)
            wind_map[i][j] = randf_range(0.0f, 1.0f);
}

/**
 * apply_environmental_bending
 * ----------------------------
 * Applica una piegatura ambientale alla lama d'erba (GrassBlade) dovuta al vento.
 *
 * Parametri:
 *  - GrassBlade* blade: puntatore alla struttura della lama d'erba da modificare.
 *
 * Restituisce:
 *  - void
 *
 * Descrizione:
 *  Calcola l'influsso del vento in base alla posizione della lama e a una direzione vento fissa,
 *  quindi incrementa la curvatura della lama in proporzione a questo fattore.
 */
void apply_environmental_bending(GrassBlade* blade) {
    // Direzione del vento lungo gli assi X e Z (valori costanti, direzione fissa)
    float wind_dir_x = -0.3f, wind_dir_z = 0.4f;

    // Calcola un fattore di influsso del vento basato sulla posizione della lama (blade)
    // Maggiore è la proiezione della posizione della lama sulla direzione del vento, maggiore la piegatura
    float factor = blade->x * wind_dir_x + blade->z * wind_dir_z;

    // Incrementa la curvatura della lama in base al fattore e a un coefficiente di intensità (0.01)
    blade->curvature += 0.01f * factor;
}

/**
 * sample_terrain_height
 * ---------------------
 * Calcola l'altezza del terreno in una posizione (x, z).
 *
 * Parametri:
 *  - float x: coordinata X nel piano orizzontale.
 *  - float z: coordinata Z nel piano orizzontale.
 *
 * Restituisce:
 *  - float: altezza del terreno in quel punto.
 *
 * Descrizione:
 *  La funzione utilizza una combinazione di seno e coseno per simulare
 *  variazioni ondulate nel terreno, con ampiezza limitata a 0.2.
 */
float sample_terrain_height(float x, float z) {
    // Calcola l'altezza come prodotto di seno e coseno scalati,
    // con ampiezza 0.2 per mantenere variazioni moderate nel terreno
    return 0.2f * sinf(x * 0.5f) * cosf(z * 0.5f);
}

/**
 * botanical_suitability
 * ---------------------
 * Valuta l'idoneità botanica di un punto (x, z) in base a diversi fattori ambientali.
 *
 * Parametri:
 *  - float x: coordinata X nel piano orizzontale.
 *  - float z: coordinata Z nel piano orizzontale.
 *
 * Restituisce:
 *  - float: valore di idoneità botanica (da 0 a 1), dove 1 indica condizioni ottimali.
 *
 * Descrizione:
 *  Combina tre fattori:
 *   1. Preferenza per altitudini moderate (vicino a 0.0), modellata come una gaussiana.
 *   2. Penalizzazione per l'ombra: zone più soleggiate sono preferite.
 *   3. Penalizzazione per la pendenza del terreno: terreno troppo inclinato è meno adatto.
 */
float botanical_suitability(float x, float z) {
    // Ottiene l'altezza del terreno in (x,z)
    float height = sample_terrain_height(x, z);

    // Preferenza per zone a mezza quota (vicino a 0.0 in altezza)
    // Usa una funzione gaussiana centrata su 0 con deviazione 0.5
    float elevation_pref = expf(-powf(height / 0.5f, 2));

    // Campiona il valore di ombra nella posizione (0 = pieno sole, 255 = piena ombra)
    float shadow = sample_shadow_mask(x, z) / 255.0f;

    // Penalizza l'ombra: più ombra, minore l'idoneità
    float shadow_penalty = 1.0f - shadow;

    // Calcola la pendenza del terreno come derivata parziale centrale
    float d_dx = 0.5f * (sample_terrain_height(x + 0.1f, z) - sample_terrain_height(x - 0.1f, z));
    float d_dz = 0.5f * (sample_terrain_height(x, z + 0.1f) - sample_terrain_height(x, z - 0.1f));

    // Magnitudine del gradiente -> pendenza del terreno
    float slope = sqrtf(d_dx * d_dx + d_dz * d_dz);

    // Penalizza fortemente le zone con pendenza elevata (curva esponenziale)
    float slope_penalty = expf(-slope * 5.0f);

    // Combina i tre fattori moltiplicandoli: preferenza per quota, poca ombra e terreno piano
    return elevation_pref * shadow_penalty * slope_penalty;
}

/**
 * generate_grass_clumps
 * ---------------------
 * Genera un numero specificato di ciuffi d’erba (NUM_CLUMPS) posizionati casualmente in un'area.
 * Per ogni ciuffo, controlla il tipo di terreno, vegetazione e idoneità botanica prima di crearlo.
 * Inizializza ogni lama d’erba del ciuffo con proprietà variabili (posizione, altezza, colore, curvatura, ecc.).
 *
 * Parametri di input: nessuno (usa variabili globali come NUM_CLUMPS e clumps)
 * Output: nessuno (modifica direttamente l’array globale di clumps)
 */
void generate_grass_clumps() {
    int created = 0;
    // Continua a creare ciuffi finché non si raggiunge il numero desiderato (NUM_CLUMPS)
    while (created < NUM_CLUMPS) {
        // Genera una posizione casuale (x,z) nell’area [-10,10]
        float x = randf_range(-10.0f, 10.0f);
        float z = randf_range(-10.0f, 10.0f);

        // Verifica il tipo di terreno in quella posizione
        int type = terrain_type(x, z);
        if (type == 0) continue; // Se terreno spoglio, ignora e rigenera

        // Verifica il tipo di vegetazione in quella posizione
        int vtype = vegetation_type(x, z);
        if (vtype == 0) continue; // Se zona arida, ignora e rigenera

        // Calcola l’idoneità botanica per crescere erba in quella posizione
        float suitability = botanical_suitability(x, z);
        if (suitability < 0.3f) continue; // Se bassa idoneità, rigenera

        // Se la posizione è valida, inizializza un nuovo ciuffo di erba
        GrassClump* clump = &clumps[created];
        clump->x = x;
        clump->z = z;
        clump->orientation = randf_range(0.0f, 2 * M_PI); // orientazione casuale
        clump->count = 5 + rand() % (MAX_BLADES_PER_CLUMP - 5); // numero lame casuale

        // Genera ogni lama d’erba nel ciuffo
        for (int i = 0; i < clump->count; i++) {
            GrassBlade* blade = &clump->blades[i];

            // Offset casuale intorno al centro del ciuffo per la posizione della lama
            float offset_x = randf_range(-0.2f, 0.2f);
            float offset_z = randf_range(-0.2f, 0.2f);
            blade->x = clump->x + offset_x;
            blade->z = clump->z + offset_z;

            // Calcola l’altezza del terreno in quella posizione per posizionare la lama sull’erba
            blade->y = sample_terrain_height(blade->x, blade->z);

            // Altezza della lama con distribuzione gaussiana e variazione legata a posizione
            blade->height = rand_gaussian(0.7f + 0.3f * sinf(blade->x * blade->z), 0.2f);

            // Curvatura casuale, ma con un minimo per evitare lame troppo dritte
            float c = randf_range(-0.25f, 0.25f);
            blade->curvature = (fabsf(c) < 0.08f) ? (c < 0 ? -0.08f : 0.08f) : c;

            // Se la curvatura è più accentuata, aumenta leggermente l’altezza
            if (fabsf(blade->curvature) > 0.1f)
                blade->height *= 1.1f;

            // Imposta spessore e segmenti (determinano la forma della lama)
            blade->thickness = 0.01f;
            blade->segments = 10 + rand() % 4;

            // Offset casuale per l’animazione del vento
            blade->wind_offset = randf_range(0, 3.14f);

            blade->shadow_factor = 1.0f;    // fattore ombra iniziale pieno
            blade->depth_in_clump = (float)i / clump->count; // profondità relativa nel ciuffo

            blade->bends.count = 0;         // nessuna piegatura iniziale
            blade->has_flower = false;      // nessun fiore di default

            // Imposta colori e possibile fiore in base al tipo di vegetazione
            if (vtype == 1) {
                // Erba secca/giallastra
                blade->height *= 0.8f; // un po' più bassa
                blade->r = 0.5f + randf_range(0.0f, 0.1f);
                blade->g = 0.5f + randf_range(0.0f, 0.1f);
                blade->b = 0.1f;
            } else if (vtype == 2) {
                // Erba verde normale
                blade->r = 0.2f + randf_range(0.0f, 0.1f);
                blade->g = 0.7f + randf_range(0.0f, 0.2f);
                blade->b = 0.05f;
            } else if (vtype == 3) {
                // Erba fertile, più verde e con fiori possibili
                blade->r = 0.2f + randf_range(0.0f, 0.1f);
                blade->g = 0.8f + randf_range(0.0f, 0.2f);
                blade->b = 0.1f;

                // Possibilità del 10% di aggiungere un fiore decorativo
                if (randf_range(0.0f, 1.0f) < 0.1f) {
                    blade->has_flower = true;
                    if (randf_range(0.0f, 1.0f) < 0.5f) {
                        // Fiore giallo
                        blade->flower_r = 1.0f;
                        blade->flower_g = 1.0f;
                        blade->flower_b = 0.0f;
                    } else {
                        // Fiore blu
                        blade->flower_r = 0.3f;
                        blade->flower_g = 0.3f;
                        blade->flower_b = 1.0f;
                    }
                }
            }

            // Applica deformazioni ambientali (vento, pieghe)
            apply_environmental_bending(blade);
        }

        // Incrementa il contatore ciuffi creati
        created++;
    }
}

/**
 * compare_blades
 * --------------
 * Funzione di confronto utilizzata per ordinare un array di puntatori a GrassBlade in base alla loro coordinata z.
 * Ordina in ordine decrescente rispetto a z: se A.z < B.z, A viene dopo B.
 *
 * Parametri di input:
 *  - a: puntatore generico a un elemento dell'array (in realtà puntatore a puntatore a GrassBlade)
 *  - b: come sopra, secondo elemento da confrontare
 *
 * Ritorno:
 *  - Intero: 1 se A.z < B.z (A dopo B), -1 altrimenti (A prima di B)
 */
int compare_blades(const void* a, const void* b) {
    // Dereferenzia i puntatori a GrassBlade
    const GrassBlade* A = *(GrassBlade**)a;
    const GrassBlade* B = *(GrassBlade**)b;

    // Se la z di A è minore di quella di B, A viene dopo B (ritorna 1)
    // Altrimenti A viene prima di B (ritorna -1)
    return (A->z < B->z) ? 1 : -1;
}

/**
 * bezier_quadratic
 * ----------------
 * Calcola la posizione di un punto sulla curva di Bézier quadratica per un parametro t dato.
 *
 * Parametri di input:
 *  - t: valore del parametro nella curva, compreso tra 0 e 1
 *  - p0: array di 3 float che rappresenta il punto di partenza (x,y,z)
 *  - p1: array di 3 float che rappresenta il punto di controllo (x,y,z)
 *  - p2: array di 3 float che rappresenta il punto finale (x,y,z)
 *
 * Parametro output:
 *  - out: array di 3 float dove viene salvata la posizione calcolata sulla curva (x,y,z)
 *
 * Formula usata:
 *  B(t) = (1-t)^2 * p0 + 2(1-t)t * p1 + t^2 * p2
 */
void bezier_quadratic(float t, float* p0, float* p1, float* p2, float* out) {
    float u = 1.0f - t;

    // Formula della curva di Bézier quadratica:
    // B(t) = (1-t)^2 * p0 + 2(1-t)t * p1 + t^2 * p2
    out[0] = u * u * p0[0] + 2 * u * t * p1[0] + t * t * p2[0];
    out[1] = u * u * p0[1] + 2 * u * t * p1[1] + t * t * p2[1];
    out[2] = u * u * p0[2] + 2 * u * t * p1[2] + t * t * p2[2];
}

/**
 * generate_dynamic_shadow_map
 * ---------------------------
 * Calcola una mappa delle ombre dinamiche basata sulle posizioni delle punte delle lame d'erba
 * proiettate sulla superficie del terreno in base alla direzione della luce.
 *
 * Input:
 *   - Nessuno (usa variabili globali come clumps, light_dir, dynamic_shadow_map)
 *
 * Output:
 *   - void (modifica la shadow map globale dynamic_shadow_map)
 *
 * Descrizione:
 *   Inizializza la shadow map con valori molto bassi e, per ogni lama d'erba,
 *   proietta la punta verso il terreno lungo la direzione della luce.
 *   Aggiorna la shadow map mantenendo l'altezza massima visibile dalla luce.
 */
void generate_dynamic_shadow_map() {
    
    // Inizializza la mappa delle ombre dinamiche con valori molto bassi,
    // così nessuna altezza è registrata all'inizio (impostata come "invisibile")
    for (int i = 0; i < SHADOW_MAP_RES; i++)
        for (int j = 0; j < SHADOW_MAP_RES; j++)
            dynamic_shadow_map[i][j] = -9999.0f;

    // Per ogni ciuffo d'erba
    for (int c = 0; c < NUM_CLUMPS; c++) {
        GrassClump* clump = &clumps[c];

        // Per ogni lama nel ciuffo
        for (int i = 0; i < clump->count; i++) {
            GrassBlade* blade = &clump->blades[i];

            // Calcola la posizione della punta della lama d’erba (posizione + altezza)
            float tip_x = blade->x;
            float tip_y = blade->y + blade->height;
            float tip_z = blade->z;

            // Proietta la punta della lama verso il terreno seguendo la direzione della luce (light_dir)
            // Calcola il tempo t necessario per raggiungere y = 0 (superficie del terreno)
            float t = (tip_y) / light_dir[1];

            // Calcola le coordinate proiettate della punta sulla superficie del terreno
            float proj_x = tip_x - light_dir[0] * t;
            float proj_z = tip_z - light_dir[2] * t;

            // Mappa le coordinate proiettate da [-10,10] a [0, SHADOW_MAP_RES] per la shadow map
            int sx = (int)((proj_x + 10.0f) / 20.0f * SHADOW_MAP_RES);
            int sz = (int)((proj_z + 10.0f) / 20.0f * SHADOW_MAP_RES);

            // Se le coordinate sono valide all'interno della mappa
            if (sx >= 0 && sx < SHADOW_MAP_RES && sz >= 0 && sz < SHADOW_MAP_RES) {
                // Aggiorna la mappa solo se la punta attuale è più alta di quella già registrata
                if (tip_y > dynamic_shadow_map[sz][sx]) {
                    dynamic_shadow_map[sz][sx] = tip_y;
                }
            }
        }
    }
}

/**
 * sample_dynamic_shadow
 * ---------------------
 * Determina il livello di ombra in una certa posizione (x, z, y) usando la mappa delle ombre dinamiche.
 *
 * Parametri di input:
 *   - float x: coordinata X nello spazio di gioco (terra)
 *   - float z: coordinata Z nello spazio di gioco (terra)
 *   - float y: altezza corrente della lama d'erba (coordinate Y)
 *
 * Output:
 *   - float: valore di ombra, 0.3 se in ombra attenuata, 1.0 se in piena luce
 *
 * Descrizione:
 *   Mappa (x, z) nella shadow map e confronta l'altezza y con quella registrata nella mappa.
 *   Se y è inferiore all'altezza dell'ombra (con una piccola tolleranza), la lama è in ombra.
 */
float sample_dynamic_shadow(float x, float z, float y) {
   
    // Calcola l'indice x nella shadow map mappando la coordinata x da [-10,10] a [0, SHADOW_MAP_RES]
    int sx = (int)((x + 10.0f) / 20.0f * SHADOW_MAP_RES);

    // Calcola l'indice z nella shadow map mappando la coordinata z da [-10,10] a [0, SHADOW_MAP_RES]
    int sz = (int)((z + 10.0f) / 20.0f * SHADOW_MAP_RES);

    // Se le coordinate calcolate escono fuori dalla shadow map, si assume che sia in piena luce (valore 1.0)
    if (sx < 0 || sx >= SHADOW_MAP_RES || sz < 0 || sz >= SHADOW_MAP_RES)
        return 1.0f;

    // Ottiene l'altezza dell'ombra dinamica registrata nella shadow map per la posizione (sz, sx)
    float shadow_height = dynamic_shadow_map[sz][sx];

    // Se l'altezza y della lama (aggiustata di 0.05 per evitare artefatti) è più bassa
    // rispetto all'altezza dell'ombra → la lama è in ombra, altrimenti è in piena luce
    // Ritorna 0.3 per ombra attenuata, 1.0 per piena luce
    return (y + 0.05f < shadow_height) ? 0.3f : 1.0f;
}

/**
 * draw_grass
 * ----------
 * Disegna tutte le lame d'erba suddividendole in bucket in base alla profondità dalla camera.
 * Applica effetti di piegatura dovuti al vento e ombre dinamiche per un rendering realistico.
 *
 * Parametri di input:
 *   - float time: tempo corrente, usato per animazioni (es. vento, piegatura)
 *
 * Output:
 *   - void (effettua il rendering delle lame d'erba direttamente)
 *
 * Descrizione:
 *   1) Organizza le lame in bucket per profondità per ottimizzare il disegno.
 *   2) Normalizza la direzione della luce.
 *   3) Per ogni lama, calcola piegatura e ombra dinamica, poi disegna la lama con curva di Bézier.
 *   4) Se presente, disegna anche un fiore sulla punta della lama.
 */
void draw_grass(float time) {
    // Svuota tutti i bucket azzerando il conteggio delle lame per ciascuno
    for (int i = 0; i < MAX_BUCKETS; i++) {
        buckets[i].count = 0;
    }

    // Inserisce ogni lama d'erba nel bucket corretto in base alla sua profondità rispetto alla camera
    for (int c = 0; c < NUM_CLUMPS; c++) {
        GrassClump* clump = &clumps[c];
        for (int i = 0; i < clump->count; i++) {
            GrassBlade* blade = &clump->blades[i];

            // Calcola la profondità della lama dalla camera
            float depth = camera_depth(blade);

            // Calcola indice bucket normalizzando la profondità su MAX_BUCKETS e range di 20 unità
            int bucket_index = (int)(depth * MAX_BUCKETS / 20.0f);

            // Limita l'indice bucket all'intervallo valido
            if (bucket_index >= MAX_BUCKETS) bucket_index = MAX_BUCKETS - 1;
            if (bucket_index < 0) bucket_index = 0;

            // Inserisce la lama nel bucket solo se non si è raggiunto il massimo numero di lame per bucket
            if (buckets[bucket_index].count < MAX_BLADES_PER_BUCKET) {
                buckets[bucket_index].blades[buckets[bucket_index].count++] = blade;
            }
        }
    }

    // Normalizza la direzione della luce per i calcoli di shading
    normalize(light_dir);

    // Disegna i bucket partendo dal più lontano (massima profondità) al più vicino
    for (int b = MAX_BUCKETS - 1; b >= 0; b--) {
        for (int i = 0; i < buckets[b].count; i++) {
            GrassBlade* blade = buckets[b].blades[i];

            // Campiona il vento in base alla posizione della lama e al tempo per simulare movimento
            float wind = sample_wind_map(blade->x, blade->z, time);

            // Aggiunge una funzione di piegatura casuale alla lama circa 1 volta ogni 10 iterazioni
            if ((rand() % 10) == 0) {
                add_bending_function(blade, time, wind);
            }

            int segments = blade->segments;                     // Numero di segmenti per la lama (per curva)
            float segment_length = blade->height / segments;    // Lunghezza di ogni segmento
            float attenuation = expf(-3.0f * blade->depth_in_clump); // Attenuazione basata sulla profondità nella chioma

            // Vettore normale standard per il calcolo dell'illuminazione (punta verso Z)
            float normal[3] = { 0.0f, 0.0f, 1.0f };

            // Calcola il prodotto scalare per ottenere l'intensità luminosa (clampa a 0 minimo)
            float dot = fmaxf(0.0f, normal[2] * light_dir[2]);

            // Campiona la probabilità di ombra dinamica in base alla posizione della lama
            float shadow_prob = sample_dynamic_shadow(blade->x, blade->z, blade->y);

            // Determina casualmente se la lama è in luce o ombra in questo frame
            bool in_light = (randf_range(0.0f, 1.0f) < shadow_prob);

            // Calcola il colore finale della lama tenendo conto di ombra, luce e attenuazione
            float r, g, b;
            compute_colored_shading(blade, dot, attenuation, in_light, &r, &g, &b, time);

            // Imposta il colore corrente per disegnare la lama
            glColor3f(r, g, b);

            // Inizia a disegnare la lama come una linea curva (line strip)
            glBegin(GL_LINE_STRIP);

            // Punto base (radice della lama)
            float p0[3] = { blade->x, blade->y, blade->z };

            // Calcola la curvatura combinata tra quella base e la piegatura dovuta al vento
            float bend = apply_bending(&blade->bends, time, 1.0f);
            float curve_amt = blade->curvature + bend;

            // Punto di controllo per la curva Bézier (curvatura intermedia)
            float p1[3] = {
                blade->x + curve_amt * blade->height * 1.5f,
                blade->y + blade->height * 0.3f,
                blade->z + curve_amt * 0.7f
            };

            // Punto finale della curva (punta della lama)
            float p2[3] = {
                blade->x + curve_amt,
                blade->y + blade->height,
                blade->z
            };

            // Disegna la curva quadratica di Bézier suddividendo la lama in segmenti
            for (int j = 0; j <= segments; j++) {
                //float t = (float)j / segments;
                float t = segment_length * j / blade->height;
                float pos[3];
                bezier_quadratic(t, p0, p1, p2, pos);  // Calcola il punto sulla curva per parametro t
                glVertex3f(pos[0], pos[1], pos[2]);    // Specifica il vertice corrente
            }

            glEnd();

            // Se la lama ha un fiore, disegna un punto colorato sulla punta
            if (blade->has_flower) {
                float tip[3];
                bezier_quadratic(1.0f, p0, p1, p2, tip);  // Ottieni la posizione finale della curva
                glPointSize(4.0f);                         // Imposta dimensione del punto
                glBegin(GL_POINTS);
                glColor3f(blade->flower_r, blade->flower_g, blade->flower_b); // Colore del fiore
                glVertex3f(tip[0], tip[1], tip[2]);       // Disegna il punto sulla punta
                glEnd();
            }
        }
    }
}

/**
 * draw_ground
 * -----------
 * Disegna un semplice piano quadrato che rappresenta il terreno.
 *
 * Il piano è disegnato sul piano XZ con altezza Y=0 e ha colore marrone scuro.
 */
void draw_ground() {
    // Imposta il colore del terreno (marrone scuro)
    glColor3f(0.4f, 0.25f, 0.1f);

    // Inizia a disegnare un quadrilatero (quad)
    glBegin(GL_QUADS);

    // Definisce i quattro vertici del quadrilatero sul piano XZ a Y=0
    glVertex3f(-12.0f, 0.0f, -12.0f);  // angolo in basso a sinistra
    glVertex3f(-12.0f, 0.0f, 12.0f);   // angolo in alto a sinistra
    glVertex3f(12.0f, 0.0f, 12.0f);    // angolo in alto a destra
    glVertex3f(12.0f, 0.0f, -12.0f);   // angolo in basso a destra

    // Fine disegno del quadrilatero
    glEnd();
}

/**
 * draw_scene
 * ----------
 * Imposta le matrici di proiezione e vista e disegna la scena completa.
 *
 * Parametri:
 *  - GLFWwindow* window: finestra corrente GLFW (per ottenere dimensioni framebuffer)
 *  - float time: tempo corrente, usato per animazioni dinamiche (es. vento erba)
 *  - float camX: posizione orizzontale della camera (asse X)
 *  - float camZ: posizione orizzontale della camera (asse Z)
 *
 * Funzionamento:
 *  1) Aggiorna la viewport e proiezione prospettica basandosi sulle dimensioni della finestra.
 *  2) Imposta la posizione e orientamento della camera con gluLookAt.
 *  3) Disegna il terreno statico.
 *  4) Disegna l'erba dinamica con animazioni.
 */
void draw_scene(GLFWwindow* window, float time, float camX, float camZ) {
    // Ottiene la dimensione corrente del framebuffer (utile per gestire il ridimensionamento finestra)
    int width, height;
    glfwGetFramebufferSize(window, &width, &height);

    // Calcola il rapporto larghezza/altezza (aspect ratio) per la proiezione prospettica
    float ratio = width / (float)height;

    // Imposta la viewport per coprire tutta la finestra corrente
    glViewport(0, 0, width, height);

    // Pulisce i buffer di colore e profondità prima di disegnare il nuovo frame
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Imposta la matrice di proiezione
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();  // Resetta la matrice di proiezione

    // Definisce una proiezione prospettica con campo visivo 45°, aspect ratio calcolato, e piano vicino/lontano
    gluPerspective(45.0, ratio, 0.1, 100.0);

    // Imposta la matrice per la modellazione e vista (ModelView)
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();  // Resetta la matrice ModelView

    // Posiziona la camera usando gluLookAt:
    // - posizione camera (camX, 2.0, camZ)
    // - punto verso cui guarda (camX, 0.5, 0.0)
    // - vettore up (0,1,0), direzione “in alto”
    gluLookAt(camX, 2.0, camZ,
              camX, 0.5, 0.0,
              0.0, 1.0, 0.0);

    // Disegna il terreno statico
    draw_ground();

    // Disegna l’erba dinamica, passando il tempo corrente per animazioni
    draw_grass(time);
}


/*    ************************************************************************ */
//  FUNZIONI PER INTERAZIONE DA TASTIERA

/**
 * process_input
 * -------------
 * Gestisce l'input da tastiera per muovere la camera nello spazio.
 *
 * Parametri:
 *  - GLFWwindow* window: puntatore alla finestra GLFW da cui leggere lo stato della tastiera.
 *
 * Funzionamento:
 *  - Se viene premuto il tasto W, la camera si sposta in avanti (asse Z diminuisce).
 *  - Se viene premuto il tasto S, la camera si sposta indietro (asse Z aumenta).
 *  - Se viene premuto il tasto A, la camera si sposta a sinistra (asse X diminuisce).
 *  - Se viene premuto il tasto D, la camera si sposta a destra (asse X aumenta).
 *
 * Nota: camX e camZ sono variabili globali che rappresentano la posizione corrente della camera.
 */
void process_input(GLFWwindow* window) {
    // Se il tasto W è premuto, sposta la camera in avanti (diminuisce camZ)
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) camZ -= 0.1f;
    
    // Se il tasto S è premuto, sposta la camera indietro (aumenta camZ)
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) camZ += 0.1f;
    
    // Se il tasto A è premuto, sposta la camera a sinistra (diminuisce camX)
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) camX -= 0.1f;
    
    // Se il tasto D è premuto, sposta la camera a destra (aumenta camX)
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) camX += 0.1f;
}

/*    ************************************************************************ */
// MAIN

int main(void) {
    
    // Inizializza il generatore di numeri casuali usando il tempo attuale
    srand((unsigned int)time(NULL));

    // Inizializza GLFW, libreria per gestione finestra e contesto OpenGL
    if (!glfwInit()) return -1;  // Se fallisce, termina con codice -1

    // Crea una finestra con dimensioni WIDTH x HEIGHT e titolo "Erba Dinamica"
    GLFWwindow* window = glfwCreateWindow(WIDTH, HEIGHT, "Particle System : Grass", NULL, NULL);
    if (!window) {
        // Se la creazione della finestra fallisce, termina GLFW e il programma
        glfwTerminate();
        return -1;
    }

    // Rende il contesto OpenGL della finestra corrente attivo
    glfwMakeContextCurrent(window);

    // Abilita il multisampling per smussare le linee
    glEnable(GL_LINE_SMOOTH);

    // Abilita il test di profondità per un corretto rendering 3D
    glEnable(GL_DEPTH_TEST);

    // Imposta la larghezza delle linee disegnate
    glLineWidth(0.6f);

    // Imposta il colore di sfondo del buffer di cancellazione (azzurro cielo)
    glClearColor(0.8f, 0.9f, 1.0f, 1.0f);

    // Genera una volta i dati statici necessari all'applicazione
    generate_shadow_mask();
    generate_terrain_map();
    generate_wind_map();
    generate_vegetation_map();
    generate_grass_clumps();

    // Ciclo principale di rendering, continua fino a chiusura finestra
    while (!glfwWindowShouldClose(window)) {
        // Gestisce l’input da tastiera o mouse
        process_input(window);

        // Ottiene il tempo trascorso dall’avvio per animazioni dinamiche
        float time = (float)glfwGetTime();

        // Aggiorna dinamicamente la mappa delle ombre in base al tempo o altre condizioni
        generate_dynamic_shadow_map();

        // Esegue il rendering della scena (terreno + erba)
        draw_scene(window, time, camX, camZ);

        // Scambia i buffer per visualizzare il frame appena renderizzato
        glfwSwapBuffers(window);

        // Gestisce gli eventi come input o ridimensionamenti
        glfwPollEvents();
    }

    // Distrugge la finestra e termina GLFW pulitamente
    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;  // Termina il programma con successo
}





































