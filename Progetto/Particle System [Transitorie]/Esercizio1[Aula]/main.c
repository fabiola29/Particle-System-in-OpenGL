/*
    particle_system.c
    di Salomone Fabiola
    (riferimento :
    William T. Reeves)
*/

/*  ************************************************************************
    DESCRIZIONE

    In questo file è presentata una possibile implementazione di un sistema di particelle
    ottimizzato per la simulazione e il rendering di oggetti come fuoco, neve,
    fuochi d'artificio, fumo e stelle.
     
*/

/*  ************************************************************************ */
// LIBRERIE
#define GL_SILENCE_DEPRECATION
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include "stdbool.h"

#include "texture.h"  //  Utility per caricare texture RGBA

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h" // Decodifica immagini

/*   ************************************************************************ */
// ENUMERAZIONI, COSTANTI E VARIABILI GLOBALI

#define MAX_PARTICLES 5000           // Numero massimo di particelle nel sistema
#define MAX_CHILDREN 64              // Numero massimo di particelle figlie per ogni emettitore
#define TARGET_FPS 60                // Frame per secondo desiderati
#define FRAME_DURATION (1.0 / TARGET_FPS) // Durata di un frame


//Tipi di particelle
typedef enum {
    PARTICLE_LAUNCH,       // Lancio iniziale (es. razzo fuochi)
    PARTICLE_EXPLOSION,    // Esplosione
    PARTICLE_FLAME,        // Fiamma
    PARTICLE_FIREWORK,     // Fuochi d'artificio
    PARTICLE_STAR,         // Stelline scintillanti
    PARTICLE_SMOKE,        // Fumo
    PARTICLE_SNOW          // Fiocchi di neve
} ParticleType;


//Forme geometriche di emissione
typedef enum {
    SHAPE_POINT,        // Punto singolo
    SHAPE_LINE,         // Linea
    SHAPE_SOFT_CIRCLE,  // Cerchio con distribuzione morbida
    SHAPE_CIRCLE_XY,    // Cerchio sul piano XY
    SHAPE_RECT_XY,      // Rettangolo sul piano XY
    SHAPE_SPHERE,       // Sfera 3D
    SHAPE_STAR          // Forma a stella
} ParticleShape;

ParticleShape current_emission_shape = SHAPE_SPHERE;  // Forma corrente di emissione

struct Particle* parent;  // Particella padre

//STRUTTURA PARTICELLA
typedef struct Particle {
    // Posizione attuale e precedente
    float x, y, z;
    float prev_x, prev_y, prev_z;

    // Velocità e accelerazione
    float vx, vy, vz;
    float ax, ay, az;

    // Durata della particella e dimensione iniziale/finale
    float life;
    float size_start, size_end;

    // Colori iniziali e finali (RGBA)
    float r_start, g_start, b_start, a_start;
    float r_end, g_end, b_end, a_end;

    int active;             // Se la particella è attiva o meno
    ParticleType type;      // Tipo di particella

    // == EMMISSIONE ==
    int is_emitter;             // Se true, genera altre particelle
    float emission_rate;        // Quante particelle genera al secondo
    float emission_accumulator; // Accumulatore frazionario per emissioni
    float origin_x, origin_y, origin_z; // Origine dell’emissione

    // Relazioni gerarchiche
    struct Particle* children[MAX_CHILDREN]; // Particelle figlie
    int child_count;                         // Numero di figli
    struct Particle* parent;                 // Particella padre

    // Aspetto visivo
    ParticleShape shape;     // Forma geometrica di emissione
    int color_easing;        // Tipo di interpolazione per il colore
    int size_easing;         // Tipo di interpolazione per la dimensione

    // Per il sorting della trasparenza rispetto alla camera
    float camera_dist;
} Particle;

//EASING TYPES
typedef enum {
    EASING_LINEAR,         // Interpolazione lineare
    EASING_SMOOTHSTEP,     // Morbida
    EASING_IN_QUAD,        // Accelera
    EASING_OUT_QUAD,       // Rallenta
    EASING_IN_CUBIC,       // Cubica in ingresso
    EASING_OUT_CUBIC,      // Cubica in uscita
    EASING_EXPONENTIAL,    // Esponenziale
    EASING_CUSTOM1         // Personalizzata
} EasingType;

float gravity = -9.8f;  // Gravità applicata sulle particelle

int f0 = 0;  // Contatore dei frame, usato per gestire generazione condizionata al tempo

// Metodo 1: generazione fissa con varianza
float InitialMeanParts = 100.0f;
float VarParts = 20.0f;

// Metodo 2: crescita lineare col tempo
float DeltaMeanParts = 0.5f;

// Metodo 3: dipende dall'area visibile dello schermo (LOD)
float InitialMeanPartssa = 0.05f;
float DeltaMeanPartssa = 0.005f;
float VarPartssa = 0.01f;
bool use_screen_area_control = false;
float screen_area = 800.0f * 600.0f;  // Area visibile (fittizia)

float light_dir[3] = { 0.3f, -1.0f, 0.4f };  // Direzione della luce (verso il basso)

float extinction_alpha_threshold = 0.02f;      // Sotto questo alpha, la particella è considerata estinta
float extinction_distance_threshold = 80.0f;   // Se troppo lontana dalla camera
float extinction_brightness_threshold = 0.02f; // Se troppo scura
float extinction_visual_intensity_threshold = 0.05f; // Se visivamente trascurabile

GLuint texFlame, texSmoke, texStar, texSnow; // Texture OpenGL per i vari effetti

//Attivazione effetti
int launch_active = 0;
int flame_active = 0;
int ground_fire_active = 0;
int snow_active = 0;
int paused = 0; // Pausa del sistema

// Stato della camera
float cam_angle_x = 0.0f, cam_angle_y = 0.0f;  // Rotazioni della camera
float cam_dist = 10.0f;                        // Distanza della camera dalla scena
float cam_pan_x = 0.0f, cam_pan_y = 0.0f;      // Panoramicamenti
float scene_scale_x = 1.0f, scene_scale_y = 1.0f;  // Zoom/scala

// Input mouse
int left_mouse_down = 0;
int right_mouse_down = 0;
double last_mouse_x = 0.0, last_mouse_y = 0.0; // Posizione del mouse

// Sistema particellare dinamico
Particle* particles = NULL;     // Lista dinamica di particelle
int particle_capacity = 0;      // Numero massimo allocato

//Orientamento particellare dinamico
float system_pitch_deg = 0.0f;  // Rotazione attorno all'asse X
float system_yaw_deg   = 0.0f;  // Rotazione attorno all'asse Y

/*    ************************************************************************ */
// PROTOTIPI

void allocate_particle_system(int capacity);
void free_particle_system(void);
GLuint loadTextureFromFile(const char *filename);
void loadTextures(void);
GLuint createCheckerTexture(int size, int color1[3], int color2[3]);

float clamp(float val, float min, float max);
float lerp(float a, float b, float t);
float apply_easing(float t, int type);
void apply_lighting(float* r, float* g, float* b, float x, float y, float z);
float non_linear_interp(float a, float b, float t, int mode);

void generate_position_from_shape(float* x, float* y, float* z, ParticleShape shape);
void apply_system_orientation(float* x, float* y, float* z);
void init_particle(Particle* p,
                   float x, float y, float z,
                   float vx, float vy, float vz,
                   float ax, float ay, float az,
                   float life,
                   float size_start, float size_end,
                   float r_start, float g_start, float b_start, float a_start,
                   float r_end, float g_end, float b_end, float a_end,
                   ParticleType type,
                   float origin_x, float origin_y, float origin_z, ParticleShape shape);
void add_child_particle(Particle* parent, Particle* child);
void emit_children(Particle* p, float dt);

void update_particle(Particle* p, float dt);
void update_particle_tree(Particle* p, float dt);
int compute_particles_this_frame(int f);
void clear_all_particles(void);
int find_inactive_particle(void);

void get_particle_color_and_size(Particle* p, float* r, float* g, float* b, float* a, float* size);
void draw_particle(Particle* p);
void draw_particle_tree(Particle* p);
void draw_particles(void);

void launch_firework(void);
void spawn_starfield(void);
void launch_three_fireworks(void);
void emit_background_fog(void);
void ignite_flame(float flame_angle);
void emit_smoke(void);
void emit_snowfall(float flame_angle);

void key_callback(GLFWwindow* w, int key, int sc, int action, int mods);
void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);
void cursor_position_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);

int compare_particle_distance(const void* a, const void* b);

/*    ************************************************************************ */
// FUNZIONI ACCESSORIE

// Inizializzazione e gestione risorse

/**
 * allocate_particle_system - Alloca memoria per il sistema di particelle
 *
 * Questa funzione alloca dinamicamente un array di particelle,
 * inizializzandolo a zero, per gestire fino a `capacity` particelle contemporaneamente.
 *
 * parametri in input --> capacity: il numero massimo di particelle che il sistema può contenere.
 *
 * ritorna in output --> void (non restituisce nulla).
 *
 * Dopo la chiamata, il puntatore globale `particles` punta all'array allocato,
 * e la variabile globale `particle_capacity` contiene la capacità massima.
 */
void allocate_particle_system(int capacity) {
    particles = (Particle*)calloc(capacity, sizeof(Particle)); // Alloca e azzera un array di particelle
    particle_capacity = capacity; // Salva la capacità massima del sistema
}

/**
 * free_particle_system - Libera la memoria allocata per il sistema di particelle
 *
 * Questa funzione libera la memoria dinamica precedentemente allocata per
 * l'array di particelle, se esiste, e resetta le variabili correlate.
 *
 * parametri in input :  nessuno
 * ritorna in output : void (non restituisce nulla)
 *
 * Dopo la chiamata:
 * - Il puntatore globale `particles` è impostato a NULL per evitare accessi invalidi.
 * - La capacità `particle_capacity` viene azzerata, indicando che non ci sono particelle allocate.
 */
void free_particle_system(void) {
    // Verifica se l'array di particelle è stato allocato
    if (particles) {
        free(particles);     // Libera la memoria allocata per le particelle
        particles = NULL;    // Evita dangling pointer impostando a NULL
    }

    particle_capacity = 0;   // Reimposta la capacità a 0 per indicare che il sistema è vuoto
}

/**
 * loadTextureFromFile - Carica una texture da un file immagine e la carica in OpenGL
 *
 * parametri in input :  filename: percorso del file immagine da caricare (supporta formati gestiti da stb_image)
 *
 * La funzione carica l'immagine da disco usando la libreria stb_image, forzando il caricamento
 * in formato RGBA (4 canali). Se il caricamento ha successo, crea una texture OpenGL 2D,
 * carica i dati nella GPU, configura i parametri di filtraggio e wrapping, e genera i mipmap.
 *
 * ritorna in output : GLuint --> l'ID della texture OpenGL appena creata, o 0 in caso di errore di caricamento.
 *
 * Nota: la texture deve essere successivamente gestita (bind/unbind) durante il rendering.
 */
GLuint loadTextureFromFile(const char *filename) {
    int width, height, channels;

    // Carica l'immagine da file usando stb_image, forzando 4 canali (RGBA)
    unsigned char *data = stbi_load(filename, &width, &height, &channels, 4);
    if (!data) {
        // Se il caricamento fallisce, stampa un messaggio di errore e ritorna 0
        fprintf(stderr, "❌ Errore: Impossibile caricare la texture %s\n", filename);
        return 0;
    }

    // Stampa conferma di caricamento con dimensioni immagine
    printf("✅ Texture %s caricata correttamente: %d x %d\n", filename, width, height);

    GLuint texID;
    glGenTextures(1, &texID);               // Genera un nuovo ID per la texture
    glBindTexture(GL_TEXTURE_2D, texID);    // Seleziona la texture come attiva (target: 2D)

    // Carica i dati immagine nella texture su GPU (formato RGBA a 8 bit per canale)
    glTexImage2D(
        GL_TEXTURE_2D,    // Tipo di texture
        0,                // Mipmap level (0 = base)
        GL_RGBA,          // Formato interno
        width, height,    // Dimensioni texture
        0,                // Border (deve essere 0)
        GL_RGBA,          // Formato dei dati sorgente
        GL_UNSIGNED_BYTE, // Tipo di dati
        data              // Puntatore ai pixel
    );

    glGenerateMipmap(GL_TEXTURE_2D); // Genera automaticamente tutti i livelli di mipmap

    // Imposta i parametri per il filtraggio della texture
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR); // Mipmap + interpolazione
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);               // Solo interpolazione

    // Imposta il comportamento di wrapping quando la texture esce dai limiti UV
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE); // Asse orizzontale
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE); // Asse verticale

    stbi_image_free(data); // Libera la memoria usata per l'immagine caricata

    return texID; // Restituisce l'ID della texture caricata
}

/**
 * loadTextures - Carica tutte le texture necessarie al sistema di particelle
 *
 * Questa funzione utilizza la funzione loadTextureFromFile per caricare dal disco
 * quattro texture diverse, assegnandone gli ID OpenGL alle variabili globali:
 *  - texFlame: texture della fiamma
 *  - texSmoke: texture del fumo
 *  - texStar: texture della stella
 *  - texSnow: texture del fiocco di neve
 *
 * Se una qualsiasi delle texture non viene caricata correttamente, la funzione stampa
 * un messaggio di errore e termina l'esecuzione del programma con exit(1).
 */
void loadTextures(void) {
    // Carica una texture di fiamma da file e assegna l'ID OpenGL a texFlame
    texFlame = loadTextureFromFile("/Users/Bia/Desktop/Progetto/texture/RadialGradient.png");

    // Carica la texture del fumo
    texSmoke = loadTextureFromFile("/Users/Bia/Desktop/Progetto/texture/smoke.png");

    // Carica la texture della stella
    texStar = loadTextureFromFile("/Users/Bia/Desktop/Progetto/texture/star.png");

    // Carica la texture del fiocco di neve
    texSnow = loadTextureFromFile("/Users/Bia/Desktop/Progetto/texture/fiocco.png");

    // Verifica che almeno texFlame sia stata caricata correttamente
    if (!texFlame || !texSmoke || !texStar || !texSnow) {
        fprintf(stderr, "Errore: una o più texture non sono state caricate correttamente\n");
        exit(1);
    }
}

/**
 * createCheckerTexture - Crea una texture a scacchiera (checkerboard) RGB in memoria e la carica su GPU
 *
 * parametri input:
 * size: dimensione della texture quadrata (size x size)
 * color1: array di 3 interi [R, G, B] per il primo colore (0-255)
 * color2: array di 3 interi [R, G, B] per il secondo colore (0-255)
 *
 * La funzione crea un'immagine a scacchiera con quadrati 8x8 pixel alternati tra color1 e color2,
 * genera una texture OpenGL, carica l'immagine e restituisce l'ID della texture.
 *
 * ritorna in output : GLuint, l'ID OpenGL della texture creata.
 */
GLuint createCheckerTexture(int size, int color1[3], int color2[3]) {
    // Alloca memoria per un'immagine RGB (3 canali per pixel).
    unsigned char *data = malloc(size * size * 3);

    // Ciclo su ogni pixel per assegnare il colore in base allo schema a scacchiera.
    for (int y = 0; y < size; y++) {
        for (int x = 0; x < size; x++) {
            // Calcola se il quadrato corrente è di un colore o dell'altro.
            int checker = ((x / 8) % 2) ^ ((y / 8) % 2);

            // Se checker è vero, usa color1, altrimenti color2.
            int *color = checker ? color1 : color2;

            // Calcola l'indice del pixel corrente nel buffer.
            int i = (y * size + x) * 3;

            // Imposta i valori RGB nel buffer dati.
            data[i + 0] = color[0]; // Rosso
            data[i + 1] = color[1]; // Verde
            data[i + 2] = color[2]; // Blu
        }
    }

    // Genera un ID per la texture OpenGL.
    GLuint texID;
    glGenTextures(1, &texID);

    // Associa l'ID generato come texture 2D corrente.
    glBindTexture(GL_TEXTURE_2D, texID);

    // Crea la texture e genera automaticamente i mipmap.
    gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGB, size, size, GL_RGB, GL_UNSIGNED_BYTE, data);

    // Imposta il filtro per il minification: interpolazione lineare con mipmap.
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);

    // Imposta il filtro per il magnification: interpolazione lineare.
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    // Libera la memoria allocata per l'immagine.
    free(data);

    // Restituisce l'ID della texture creata.
    return texID;
}


//Utility generali

/**
 * clamp - Restringe un valore float all'interno di un intervallo definito
 *
 * parametri input :
 * val: il valore da limitare
 * min: limite inferiore dell'intervallo
 * max: limite superiore dell'intervallo
 *
 * ritorna in output:
 * - min se val è minore di min
 * - max se val è maggiore di max
 * - altrimenti val stesso se è già nell'intervallo [min, max]
 */
float clamp(float val, float min, float max) {
    if (val < min) return min;  // Se val è minore di min, restituisce min
    if (val > max) return max;  // Se val è maggiore di max, restituisce max
    return val;                 // Altrimenti restituisce val stesso
}

/**
 * lerp - Interpolazione lineare tra due valori float
 *
 * parametri input :
 * a: valore iniziale
 * b: valore finale
 * t: parametro di interpolazione (da 0.0 a 1.0)
 *
 * Restituisce in output : un valore interpolato linearmente tra a e b.
 *
 * Quando t=0 restituisce a, quando t=1 restituisce b,
 * valori intermedi restituiscono un punto proporzionale tra i due.
 */
float lerp(float a, float b, float t) {
    return a + t * (b - a);    // Calcola un valore proporzionale a t tra a e b
}

/**
 * apply_easing - Applica una funzione di easing a un valore t
 *
 * parametri in input :
 * t: valore di interpolazione (normalmente tra 0.0 e 1.0)
 * type: tipo di easing da applicare (costante definita)
 *
 * ritorna in output :  il valore t modificato dalla funzione di easing selezionata,
 * utile per animazioni con variazioni non lineari, come accelerazioni o rallentamenti.
 */
float apply_easing(float t, int type) {
    switch (type) {
        case EASING_LINEAR:
            return t;  // Easing lineare: t invariato
        case EASING_SMOOTHSTEP:
            // Funzione Smoothstep per transizione più morbida (accelera e decelera)
            return t * t * (3 - 2 * t);
        case EASING_IN_QUAD:
            // Easing in quadratico: accelerazione lenta all'inizio
            return t * t;
        case EASING_OUT_QUAD:
            // Easing out quadratico: rallentamento verso la fine
            return 1 - (1 - t) * (1 - t);
        case EASING_IN_CUBIC:
            // Easing in cubico: accelerazione ancora più lenta all'inizio
            return t * t * t;
        case EASING_OUT_CUBIC: {
            // Easing out cubico: rallentamento più morbido verso la fine
            float u = 1 - t;
            return 1 - u * u * u;
        }
        default:
            return t;  // Default: nessun easing, ritorna t invariato
    }
}

/**
 * Applica un effetto di illuminazione semplice su un colore RGB in base alla posizione 3D del punto.
 *
 * parametri in input :
 * r    Puntatore al valore del canale rosso (verrà modificato dalla funzione).
 * g    Puntatore al valore del canale verde (verrà modificato dalla funzione).
 * b    Puntatore al valore del canale blu (verrà modificato dalla funzione).
 * x    Coordinata X del punto nello spazio 3D.
 * y    Coordinata Y del punto nello spazio 3D.
 * z    Coordinata Z del punto nello spazio 3D.
 *
 * La funzione calcola l’intensità luminosa applicata al colore originale in base alla
 * direzione della luce (fissa e approssimata) e alla distanza del punto dall’origine.
 * Modula i valori RGB per simulare l’illuminazione e l’attenuazione dovuta alla distanza.
 *
 * Non restituisce nulla, ma modifica i valori puntati da r, g e b direttamente.
 */
void apply_lighting(float* r, float* g, float* b, float x, float y, float z) {
    // Direzione della luce (normalizzata approssimativamente)
    float light_dir[3] = {0.5f, 1.0f, 0.5f};
    // Normale della superficie (puntando verso l'alto)
    float norm[3] = {0.0f, 1.0f, 0.0f};
    
    // Calcola il prodotto scalare tra la luce e la normale (quanto la luce "illumina" la superficie)
    float dot = fmaxf(0.0f, light_dir[0]*norm[0] + light_dir[1]*norm[1] + light_dir[2]*norm[2]);
    
    // Calcola la distanza del punto dall'origine
    float dist = sqrtf(x*x + y*y + z*z);
    // Calcola l'attenuazione della luce in base alla distanza (più lontano, meno luce)
    float attenuation = 1.0f / (1.0f + 0.05f * dist * dist);
    
    // Applica l'effetto di illuminazione e attenuazione ai canali RGB
    *r *= dot * attenuation;
    *g *= dot * attenuation;
    *b *= dot * attenuation;
}

/**
 * Esegue un'interpolazione non lineare tra due valori a e b, usando diversi tipi di easing.
 *
 * parametri in input :
 * a     Valore iniziale dell'interpolazione.
 * b     Valore finale dell'interpolazione.
 * t     Parametro di interpolazione (normalmente tra 0.0 e 1.0).
 * mode  Modalità di easing da applicare:
 *         1 = Ease-in quadratico (partenza lenta, accelerazione)
 *         2 = Ease-out quadratico (partenza veloce, rallentamento finale)
 *         3 = Ease-in-out con smoothstep (transizione morbida)
 *         4 = Decadimento esponenziale (transizione rapida all'inizio, lenta alla fine)
 *         default = interpolazione lineare semplice
 *
 * ritronain output : Valore interpolato tra a e b, modificato dalla funzione di easing scelta.
 */
float non_linear_interp(float a, float b, float t, int mode) {
    switch (mode) {
        case 1: // Ease-in quadratico (partenza lenta, accelerazione)
            t = t * t;
            break;
        case 2: // Ease-out quadratico (partenza veloce, rallentamento finale)
            t = 1.0f - (1.0f - t) * (1.0f - t);
            break;
        case 3: // Ease-in-out con smoothstep (transizione morbida)
            t = t * t * (3.0f - 2.0f * t);
            break;
        case 4: // Decadimento esponenziale (transizione rapida all'inizio, lenta alla fine)
            t = 1.0f - expf(-4.0f * t);
            break;
        default: // Interpolazione lineare semplice
            break;
    }
    // Calcola e ritorna il valore interpolato tra a e b usando il nuovo t modificato dall'easing
    return a + (b - a) * t;
}

//Emissione e inizializzazione particelle

/**
 * Genera una posizione casuale (x, y, z) per una particella,
 * in base alla forma geometrica specificata.
 *
 * parametri in input :
 * x      Puntatore alla coordinata x da impostare.
 * y      Puntatore alla coordinata y da impostare.
 * z      Puntatore alla coordinata z da impostare.
 * shape  Tipo di forma (ParticleShape) che determina la distribuzione spaziale:
 *          - SHAPE_CIRCLE_XY: posizioni casuali su un cerchio nel piano XY
 *          - SHAPE_RECT_XY: posizioni casuali dentro un rettangolo nel piano XY
 *          - SHAPE_SPHERE: posizioni casuali all'interno di una sfera 3D
 *          - SHAPE_POINT: posizione fissa all'origine (0,0,0)
 *
 * ritorna in output --> Void (modifica direttamente le variabili puntate da x, y, z)
 */
void generate_position_from_shape(float* x, float* y, float* z, ParticleShape shape) {
    switch (shape) {
        case SHAPE_CIRCLE_XY: {
            // Genera un angolo casuale tra 0 e 2*PI
            float angle = ((float)rand() / RAND_MAX) * 2.0f * M_PI;
            // Genera un raggio casuale con distribuzione uniforme sull'area (sqrt per uniformità)
            float radius = ((float)rand() / RAND_MAX);
            radius = sqrtf(radius) * 0.5f;  // scala il raggio a 0.5
            
            // Calcola la posizione x e y sulla circonferenza del cerchio nel piano XY
            *x = cosf(angle) * radius;
            *y = sinf(angle) * radius;
            *z = 0.0f;  // z è zero poiché è un cerchio nel piano XY
            break;
        }

        case SHAPE_RECT_XY: {
            // Dimensioni normalizzate del rettangolo (larghezza e altezza)
            float w = 1.0f, h = 0.6f;
            
            // Genera coordinate x e y casuali dentro il rettangolo centrato sull'origine
            *x = ((float)rand() / RAND_MAX - 0.5f) * w;
            *y = ((float)rand() / RAND_MAX - 0.5f) * h;
            *z = 0.0f;  // piano XY
            break;
        }

        case SHAPE_SPHERE: {
            // Angolo azimutale (0 a 2PI)
            float theta = ((float)rand() / RAND_MAX) * 2.0f * M_PI;
            // Angolo zenitale (0 a PI)
            float phi   = ((float)rand() / RAND_MAX) * M_PI;
            // Raggio con distribuzione uniforme sul volume della sfera (radice cubica per uniformità)
            float r     = cbrtf((float)rand() / RAND_MAX) * 0.5f;
            
            // Conversione da coordinate sferiche a cartesiane
            *x = r * sinf(phi) * cosf(theta);
            *y = r * sinf(phi) * sinf(theta);
            *z = r * cosf(phi);
            break;
        }

        case SHAPE_POINT:
        default:
            // Punto singolo all'origine (nessuna variazione di posizione)
            *x = 0.0f; *y = 0.0f; *z = 0.0f;
            break;
    }
}

/**
 * Applica la rotazione spaziale alle coordinate di un punto (x, y, z)
 * in base all'orientamento del sistema definito dagli angoli pitch e yaw.
 *
 * - pitch: rotazione attorno all'asse X (in gradi)
 * - yaw: rotazione attorno all'asse Y (in gradi)
 *
 * Le rotazioni vengono effettuate in ordine: prima pitch (X), poi yaw (Y).
 *
 * parametri in input :
 * x  Puntatore alla coordinata x del punto da ruotare (modificata in place)
 * y  Puntatore alla coordinata y del punto da ruotare (modificata in place)
 * z  Puntatore alla coordinata z del punto da ruotare (modificata in place)
 *
 * ritorna in output  --> Void (modifica direttamente i valori puntati da x, y, z)
 */
void apply_system_orientation(float* x, float* y, float* z) {
    // Converti gli angoli da gradi a radianti
    float pitch = system_pitch_deg * M_PI / 180.0f;  // rotazione attorno all'asse X
    float yaw   = system_yaw_deg   * M_PI / 180.0f;  // rotazione attorno all'asse Y

    // Rotazione attorno all'asse X (pitch)
    // Nuove coordinate y e z calcolate tramite la matrice di rotazione X
    float y1 = *y * cosf(pitch) - *z * sinf(pitch);
    float z1 = *y * sinf(pitch) + *z * cosf(pitch);
    *y = y1;
    *z = z1;

    // Rotazione attorno all'asse Y (yaw)
    // Nuove coordinate x e z calcolate tramite la matrice di rotazione Y
    float x1 = *x * cosf(yaw)  + *z * sinf(yaw);
    float z2 = -*x * sinf(yaw) + *z * cosf(yaw);
    *x = x1;
    *z = z2;
}

/**
 * Inizializza una particella assegnando tutti i suoi valori di stato iniziale.
 *
 * parametri in input :
 * p          Puntatore alla struttura Particle da inizializzare
 * x, y, z    Posizione iniziale della particella
 * vx, vy, vz Velocità iniziale
 * ax, ay, az Accelerazione applicata alla particella
 * life       Durata di vita residua della particella (in secondi o frame)
 * size_start Dimensione iniziale della particella
 * size_end   Dimensione finale della particella (quando la vita termina)
 * r_start, g_start, b_start, a_start  Colore iniziale RGBA della particella
 * r_end, g_end, b_end, a_end          Colore finale RGBA della particella
 * type       Tipo di particella (ad esempio fiamma, fumo, ecc)
 * origin_x, origin_y, origin_z        Posizione di origine del sistema particellare
 * shape      Forma geometrica usata per generare la posizione della particella
 *
 * ritorna in output : void
 *
 * La funzione non restituisce valore ma modifica in-place i campi della struttura Particle puntata da p.
 * Imposta anche lo stato della particella come attivo.
 */
void init_particle(Particle* p,
                   float x, float y, float z,                // posizione iniziale
                   float vx, float vy, float vz,             // velocità iniziale
                   float ax, float ay, float az,             // accelerazione
                   float life,                              // durata di vita (tempo residuo)
                   float size_start, float size_end,        // dimensione iniziale e finale della particella
                   float r_start, float g_start, float b_start, float a_start,  // colore iniziale (RGBA)
                   float r_end, float g_end, float b_end, float a_end,          // colore finale (RGBA)
                   ParticleType type,                        // tipo di particella (es. fiamma, fumo, ecc)
                   float origin_x, float origin_y, float origin_z,  // posizione d'origine sistema
                   ParticleShape shape)                      // forma in cui generare la posizione della particella
{
    // Assegna i valori passati ai membri della struttura Particle
    p->x = x; p->y = y; p->z = z;
    p->vx = vx; p->vy = vy; p->vz = vz;
    p->ax = ax; p->ay = ay; p->az = az;
    p->life = life;
    p->size_start = size_start;
    p->size_end = size_end;

    // Colore iniziale
    p->r_start = r_start; p->g_start = g_start; p->b_start = b_start; p->a_start = a_start;

    // Colore finale
    p->r_end = r_end; p->g_end = g_end; p->b_end = b_end; p->a_end = a_end;

    // Stato attivo (1 = attiva, 0 = inattiva)
    p->active = 1;

    // Tipo e forma particella
    p->type = type;
    p->origin_x = origin_x;
    p->origin_y = origin_y;
    p->origin_z = origin_z;
    p->shape = shape;
}

/**
 * Aggiunge una particella figlia all'elenco dei figli di una particella genitore.
 *
 * parametri in input :
 * parent Puntatore alla particella genitore a cui aggiungere la figlia
 * child  Puntatore alla particella figlia da aggiungere
 *
 * La funzione verifica che i puntatori non siano NULL e che il genitore non abbia
 * già raggiunto il numero massimo di figli (MAX_CHILDREN). Se tutto è corretto,
 * aggiunge la particella figlia all'array di figli del genitore e imposta il
 * puntatore al genitore nella particella figlia.
 *
 * Non restituisce valore.
 */
void add_child_particle(Particle* parent, Particle* child) {
    // Verifica che i puntatori non siano NULL e che il genitore non abbia superato il numero massimo di figli
    if (!parent || !child || parent->child_count >= MAX_CHILDREN) return;

    // Aggiunge la particella figlia all'array dei figli del genitore
    parent->children[parent->child_count++] = child;

    // Imposta il puntatore al genitore nella particella figlia
    child->parent = parent;
}

/**
 * Emette nuove particelle figlie da una particella emettitrice attiva.
 *
 * parametri in input :
 * p  Puntatore alla particella emettitrice
 * dt Delta time (intervallo temporale trascorso dall'ultimo aggiornamento)
 *
 * La funzione controlla se la particella `p` è un emettitore attivo e accumula
 * l'emissione in base al rate e al delta time. Quando l'accumulatore raggiunge
 * o supera 1, genera nuove particelle figlie con velocità casuali, inizializza
 * le loro proprietà, imposta easing per colore e dimensione, disabilita la loro
 * capacità di emettere ulteriori particelle, e le aggiunge all'elenco dei figli
 * della particella genitore. Se non ci sono particelle inattive disponibili,
 * l'emissione si interrompe.
 */
void emit_children(Particle* p, float dt) {
    // Se la particella non è un emettitore o non è attiva, non fare nulla
    if (!p->is_emitter || !p->active) return;

    // Incrementa l'accumulatore di emissione in base al rate e al delta time
    p->emission_accumulator += p->emission_rate * dt;

    // Finché l'accumulatore è >= 1, emetti nuove particelle figlie
    while (p->emission_accumulator >= 1.0f) {
        // Trova un indice di particella inattiva da riutilizzare
        int idx = find_inactive_particle();
        if (idx < 0) break; // Se non ce ne sono, esci dal ciclo

        // Calcola velocità casuali per la nuova particella figlia (piccolo range intorno allo zero)
        float vx = ((rand() / (float)RAND_MAX) - 0.5f) * 0.2f;
        float vy = ((rand() / (float)RAND_MAX)) * 0.4f;
        float vz = ((rand() / (float)RAND_MAX) - 0.5f) * 0.2f;

        // Imposta il tipo e il colore di default per la particella figlia (esplosione arancione)
        ParticleType child_type = PARTICLE_EXPLOSION;
        float r = 1.0f, g = 0.5f, b = 0.0f;

        // Modifica il tipo e colore in base al tipo del genitore
        if (p->type == PARTICLE_FLAME) {
            r = 1.0f; g = 0.3f; b = 0.0f;
            child_type = PARTICLE_FLAME;
        } else if (p->type == PARTICLE_FIREWORK) {
            r = p->r_start; g = p->g_start; b = p->b_start;
            child_type = PARTICLE_STAR;
        }

        // Inizializza la nuova particella figlia con posizione, velocità, accelerazione, durata e colori
        init_particle(&particles[idx],
            p->x, p->y, p->z,
            vx, vy, vz,
            0.0f, gravity * 0.05f, 0.0f,
            1.5f,
            0.2f, 0.1f,
            r, g, b, 0.8f,
            r, g, b, 0.0f,
            child_type,
            p->x, p->y, p->z,
            SHAPE_SOFT_CIRCLE);

        // Imposta gli easing per colore e dimensione per la particella figlia
        particles[idx].color_easing = EASING_SMOOTHSTEP;
        particles[idx].size_easing = EASING_OUT_CUBIC;

        // Questa particella figlia non emette ulteriori particelle
        particles[idx].is_emitter = 0;

        // Aggiungi la particella figlia all'elenco dei figli del genitore
        add_child_particle(p, &particles[idx]);

        // Riduci l'accumulatore di emissione di 1 per indicare che una particella è stata emessa
        p->emission_accumulator -= 1.0f;
    }
}


//Aggiornamento particelle

/**
 * Aggiorna lo stato di una particella nel sistema.
 *
 * parametri in input :
 * p   Puntatore alla particella da aggiornare
 * dt  Delta time (intervallo temporale trascorso dall'ultimo aggiornamento)
 *
 * Funzione che aggiorna posizione, velocità e stato di vita della particella `p`.
 * - Calcola la posizione precedente "allungata" in base alla velocità per effetti grafici (motion blur).
 * - Aggiorna la velocità e la posizione integrando l'accelerazione.
 * - Riduce la durata di vita residua della particella e la disattiva se scaduta.
 * - Gestisce un evento speciale per particelle di tipo PARTICLE_LAUNCH, che esplodono generando 500 figli.
 * - Applica criteri di estinzione basati su trasparenza, distanza, intensità visiva e luminosità.
 * - Se la particella è un emettitore, genera figli periodicamente con caratteristiche basate sul tipo.
 * - Aggiorna ricorsivamente tutte le particelle figlie attive.
 *
 * Non restituisce valori; modifica direttamente la particella e i suoi figli.
 */
void update_particle(Particle* p, float dt) {
    if (!p->active) return;  // Se la particella non è attiva, esci

    // Calcola la velocità istantanea (modulo del vettore velocità)
    float speed = sqrtf(p->vx*p->vx + p->vy*p->vy + p->vz*p->vz);

    // Scala proporzionale alla velocità, limitata a un massimo di 2.0 (per effetto di allungamento)
    float scale = fminf(speed * 0.1f, 2.0f);

    // Calcola la posizione precedente "allungata" indietro lungo la direzione del moto,
    // utile per effetti grafici come il motion blur o la scia
    p->prev_x = p->x - p->vx * scale;
    p->prev_y = p->y - p->vy * scale;
    p->prev_z = p->z - p->vz * scale;

    // Aggiorna la velocità con l'accelerazione (integrazione semplice)
    p->vx += p->ax * dt;
    p->vy += p->ay * dt;
    p->vz += p->az * dt;

    // Aggiorna la posizione con la velocità aggiornata
    p->x += p->vx * dt;
    p->y += p->vy * dt;
    p->z += p->vz * dt;

    // Riduci la vita della particella (decadimento)
    p->life -= dt;

    // Evento speciale per particelle di tipo PARTICLE_LAUNCH: quando la velocità verticale scende sotto la soglia,
    // la particella esplode emettendo 500 particelle figlie in tutte le direzioni
    if (p->type == PARTICLE_LAUNCH && p->vy <= 0.1f) {
        for (int i = 0; i < 500; i++) {
            int idx = find_inactive_particle();
            if (idx < 0) break;

            // Angoli casuali per emissione sferica
            float theta = ((float)rand() / RAND_MAX) * 2.0f * M_PI;
            float phi = ((float)rand() / RAND_MAX) * M_PI;

            // Velocità casuale con intensità tra 2.0 e 2.8
            float speed = 2.0f + ((float)rand() / RAND_MAX) * 0.8f;

            // Vettore direzione sferica
            float dx = sinf(phi) * cosf(theta);
            float dy = cosf(phi);
            float dz = sinf(phi) * sinf(theta);

            // Inizializza la nuova particella (stellina azzurra)
            init_particle(&particles[idx],
                p->x, p->y, p->z,
                dx * speed, dy * speed, dz * speed,
                0.0f, gravity * 0.1f, 0.0f,
                2.0f,
                0.08f, 0.08f,
                0.078f, 0.914f, 1.0f, 1.0f,
                0.078f, 0.914f, 1.0f, 0.0f,
                PARTICLE_STAR,
                p->x, p->y, p->z,
                SHAPE_POINT);

            // Imposta la posizione precedente per evitare artefatti grafici
            particles[idx].prev_x = p->x;
            particles[idx].prev_y = p->y;
            particles[idx].prev_z = p->z;
        }

        // Disattiva la particella "lancio" esplosa
        p->active = 0;
        return;
    }

    // Se la vita è terminata, disattiva la particella
    if (p->life <= 0.0f) {
        p->active = 0;
        return;
    }

    // --- Estinzione avanzata basata su vari parametri visivi e di posizione ---

    float r, g, b, a, size;
    get_particle_color_and_size(p, &r, &g, &b, &a, &size);

    // 1. Troppa trasparenza? Estingui
    if (a < extinction_alpha_threshold) {
        p->active = 0;
        return;
    }

    // 2. Troppo lontana? Estingui
    float dist2 = p->x * p->x + p->y * p->y + p->z * p->z;
    if (sqrtf(dist2) > extinction_distance_threshold) {
        p->active = 0;
        return;
    }

    // 3. Intensità visiva troppo bassa? Estingui
    float visual_intensity = (r + g + b) * a;
    if (visual_intensity < extinction_visual_intensity_threshold) {
        p->active = 0;
        return;
    }

    // 4. Luminosità troppo bassa? Estingui
    float brightness = (r + g + b) / 3.0f;
    if (brightness < extinction_brightness_threshold) {
        p->active = 0;
        return;
    }

    // --- Emissione figli se la particella è un emettitore e ha emission_rate > 0 ---

    if (p->is_emitter && p->emission_rate > 0.0f) {
        p->emission_accumulator += p->emission_rate * dt;

        while (p->emission_accumulator >= 1.0f && p->child_count < MAX_CHILDREN) {
            int idx = find_inactive_particle();
            if (idx < 0) break;

            // Velocità casuali per il figlio
            float vx = ((rand() / (float)RAND_MAX) - 0.5f) * 0.2f;
            float vy = ((rand() / (float)RAND_MAX)) * 0.4f;
            float vz = ((rand() / (float)RAND_MAX) - 0.5f) * 0.2f;

            // Imposta tipo, forma e colore figli in base al tipo genitore
            ParticleType child_type = PARTICLE_EXPLOSION;
            ParticleShape shape = SHAPE_SOFT_CIRCLE;

            float r = 0.4f, g = 0.4f, b = 0.4f; // default grigio

            if (p->type == PARTICLE_FLAME) {
                child_type = PARTICLE_SMOKE;
                shape = SHAPE_SOFT_CIRCLE;
                r = g = b = 0.3f; // fumo scuro
            } else if (p->type == PARTICLE_FIREWORK) {
                child_type = PARTICLE_STAR;
                shape = SHAPE_POINT;
                r = 1.0f; g = 0.8f; b = 0.2f; // giallo scintilla
            } else if (p->type == PARTICLE_LAUNCH) {
                child_type = PARTICLE_EXPLOSION;
                shape = SHAPE_LINE;
                r = 1.0f; g = 0.4f; b = 0.1f; // scia rosso-arancio
            }

            // Inizializza il figlio
            init_particle(&particles[idx],
                p->x, p->y, p->z,
                vx, vy, vz,
                0.0f, gravity * 0.05f, 0.0f,
                2.0f,
                0.2f, 0.4f,
                0.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 0.0f,
                child_type,
                p->x, p->y, p->z,
                shape);

            // Imposta easing per colore e dimensione
            particles[idx].color_easing = EASING_SMOOTHSTEP;
            particles[idx].size_easing = EASING_OUT_CUBIC;

            // Aggiungi figlio alla lista del genitore
            add_child_particle(p, &particles[idx]);

            p->emission_accumulator -= 1.0f;
        }
    }

    // --- Aggiorna ricorsivamente tutti i figli attivi ---
    for (int i = 0; i < p->child_count; i++) {
        Particle* child = p->children[i];
        if (!child->active) continue;
        update_particle(child, dt);
    }
}

/**
 * Aggiorna ricorsivamente una particella e tutte le sue particelle figlie.
 *
 * parametri in input :
 * p   Puntatore alla particella da aggiornare
 * dt  Delta time (intervallo temporale trascorso dall'ultimo aggiornamento)
 *
 * La funzione verifica che la particella esista e sia attiva, quindi:
 * - Aggiorna la particella corrente chiamando update_particle()
 * - Richiama se stessa ricorsivamente per aggiornare tutte le particelle figlie
 *   contenute nell'array p->children
 *
 * Questo permette di propagare l'aggiornamento lungo l'intero albero di particelle.
 */
void update_particle_tree(Particle* p, float dt) {
    // Se il puntatore è nullo o la particella non è attiva, esci subito
    if (!p || !p->active) return;

    // Aggiorna lo stato della particella corrente (posizione, vita, ecc.)
    update_particle(p, dt);

    // Per ogni figlio della particella corrente
    for (int i = 0; i < p->child_count; i++) {
        // Aggiorna ricorsivamente l'intero albero di particelle figlie
        update_particle_tree(p->children[i], dt);
    }
}

/**
 * Calcola dinamicamente il numero di particelle da generare nel frame corrente.
 *
 * parametri in input :
 * f Numero del frame corrente
 *
 * ritorna in output : Numero di particelle da generare in questo frame (intero non negativo)
 *
 * La funzione può operare in due modalità:
 * 1. Senza controllo area schermo (use_screen_area_control == false):
 *    - Calcola una media dinamica basata su un valore iniziale e una variazione lineare
 *      legata al numero di frame trascorsi (f - f0).
 *    - Aggiunge un rumore casuale con ampiezza basata sulla varianza VarParts.
 *    - Restituisce il valore finale come intero, assicurandosi che sia >= 0.
 *
 * 2. Con controllo area schermo (use_screen_area_control == true):
 *    - Calcola la media dinamica usando parametri specifici (InitialMeanPartssa, DeltaMeanPartssa, VarPartssa).
 *    - Aggiunge rumore casuale come sopra.
 *    - Moltiplica il risultato per l'area dello schermo (screen_area) per adattare
 *      il numero di particelle in base alla dimensione o risoluzione attuale.
 *    - Restituisce il valore finale come intero, assicurandosi che sia >= 0.
 *
 * Questo metodo permette di modulare dinamicamente il carico di particelle
 * generato per ottimizzare le prestazioni o ottenere effetti visivi variabili.
 */
int compute_particles_this_frame(int f) {
    float mean, variance;

    if (!use_screen_area_control) {
        // Calcola la media dinamica del numero di particelle senza controllo area schermo
        mean = InitialMeanParts + DeltaMeanParts * (f - f0);
        variance = VarParts;

        // Aggiunge un rumore casuale nella variazione per simulare fluttuazioni
        float noise = ((rand() / (float)RAND_MAX) * 2.0f - 1.0f) * variance;

        // Numero finale di particelle, media più rumore
        float n = mean + noise;

        // Assicura che il numero di particelle non sia negativo
        return (int)fmaxf(0.0f, n);
    } else {
        // Calcola la media dinamica del numero di particelle con controllo area schermo
        mean = InitialMeanPartssa + DeltaMeanPartssa * (f - f0);
        variance = VarPartssa;

        // Rumore casuale come sopra
        float noise = ((rand() / (float)RAND_MAX) * 2.0f - 1.0f) * variance;

        // Scala il numero di particelle in base all'area dello schermo
        float n = (mean + noise) * screen_area;

        // Assicura che il numero di particelle non sia negativo
        return (int)fmaxf(0.0f, n);
    }
}

/**
 * Disattiva tutte le particelle presenti nel sistema.
 *
 * Imposta lo stato di ogni particella come inattivo (active = 0)
 * e resetta il conteggio dei figli associati (child_count = 0).
 *
 * Questo è utile per azzerare completamente il sistema particellare,
 * ad esempio quando si vuole riavviare l'effetto o liberare risorse.
 *
 * Non prende parametri e non restituisce valori.
 */
void clear_all_particles(void) {
    for (int i = 0; i < particle_capacity; i++) {
        particles[i].active = 0;      // Segna la particella come inattiva
        particles[i].child_count = 0; // Resetta il numero di figli associati
    }
}

/**
 * Cerca la prima particella inattiva nell'array globale di particelle.
 *
 * Scorre l'array `particles` fino a trovare una particella con `active == 0`,
 * che indica che è inattiva e quindi disponibile per essere riutilizzata.
 *
 * Ritorna l'indice della prima particella inattiva trovata.
 * Se tutte le particelle sono attive, ritorna -1.
 *
 * ritorna in output : int Indice della particella inattiva, oppure -1 se nessuna disponibile.
 */
int find_inactive_particle(void) {
    for (int i = 0; i < particle_capacity; i++) {
        if (!particles[i].active) return i;
    }
    return -1; // Nessuna particella inattiva trovata
}

//Disegno particelle

/**
 * Calcola il colore e la dimensione correnti di una particella in base al suo tempo di vita residuo,
 * applicando interpolazioni non lineari per un effetto visivo più naturale e una simulazione di illuminazione.
 *
 * parametri in input :
 *  - Particle* p: puntatore alla particella di cui calcolare colore e dimensione
 *  - float* r, *g, *b, *a: puntatori ai valori di colore rosso, verde, blu e alpha (trasparenza) da aggiornare
 *  - float* size: puntatore alla dimensione della particella da aggiornare
 *
 * PROCESSO:
 *  - Calcola il parametro di interpolazione 't' normalizzando il tempo di vita residuo su un intervallo di 2.5 secondi
 *  - Interpola i canali colore RGB usando una funzione smoothstep per transizioni morbide
 *  - Interpola il canale alpha con un ease-out quadratico per un fade-out naturale
 *  - Interpola la dimensione con un ease-in quadratico per un effetto di riduzione progressiva
 *  - Applica un effetto di illuminazione basato sulla posizione della particella, modulando i valori RGB
 *
 * ritorna in output:
 *  - Aggiorna i valori puntati da r, g, b, a e size con i valori interpolati e illuminati correnti
 */
void get_particle_color_and_size(Particle* p, float* r, float* g, float* b, float* a, float* size) {
    // Calcola il parametro di interpolazione 't' basato sul tempo di vita residuo (normalizzato su 2.5 secondi)
    float t = 1.0f - fmaxf(0.0f, p->life) / 2.5f;

    // Clamp di 't' nell'intervallo [0, 1]
    t = fminf(1.0f, fmaxf(0.0f, t));

    // Interpolazione non lineare (smoothstep) per il colore rosso tra valore iniziale e finale
    *r = non_linear_interp(p->r_start, p->r_end, t, 3); // tipo 3 = smoothstep

    // Interpolazione non lineare (smoothstep) per il colore verde
    *g = non_linear_interp(p->g_start, p->g_end, t, 3);

    // Interpolazione non lineare (smoothstep) per il colore blu
    *b = non_linear_interp(p->b_start, p->b_end, t, 3);

    // Interpolazione con ease-out (fade-out più rapido) per il canale alpha (trasparenza)
    *a = non_linear_interp(p->a_start, p->a_end, t, 2); // tipo 2 = ease-out quadratico

    // Interpolazione con ease-in per la dimensione della particella (shrink)
    *size = non_linear_interp(p->size_start, p->size_end, t, 1); // tipo 1 = ease-in quadratico

    // Applica l’illuminazione alla particella, modificando i valori RGB in base alla posizione
    apply_lighting(r, g, b, p->x, p->y, p->z);
}

/**
 * Disegna una singola particella nel contesto OpenGL, scegliendo il metodo di rendering
 * in base alla forma della particella e applicando colore, dimensione e trasparenza
 * interpolati in base allo stato attuale della particella.
 *
 * p Puntatore alla particella da disegnare
 */
void draw_particle(Particle* p) {
    if (!p->active) return;  // Non disegnare particelle inattive

    float r, g, b, a, size;
    get_particle_color_and_size(p, &r, &g, &b, &a, &size);  // Ottieni colore e dimensione interpolati

    // Rende completamente trasparenti le particelle figlie (non disegnate direttamente)
    if (p->parent != NULL) {
        a = 0.0f;
    }

    glColor4f(r, g, b, a);  // Imposta il colore con alpha

    switch (p->shape) {
        case SHAPE_POINT:
            // Punto semplice, dimensione relativa alla size della particella
            glPointSize(size * 80.0f);
            glBegin(GL_POINTS);
            glVertex3f(p->x, p->y, p->z);
            glEnd();
            break;

        case SHAPE_LINE:
            // Linea morbida (con smoothing) tra la posizione precedente e quella attuale
            glEnable(GL_LINE_SMOOTH);
            glLineWidth(size * 100.0f);
            glBegin(GL_LINES);
            glVertex3f(p->prev_x, p->prev_y, p->prev_z);
            glVertex3f(p->x,      p->y,      p->z);
            glEnd();
            break;

        case SHAPE_SOFT_CIRCLE: {
            // Particella con texture morbida (es. fumo, fuoco, neve, stelle)
            glEnable(GL_TEXTURE_2D);

            // Seleziona la texture in base al tipo di particella
            GLuint tex = (p->type == PARTICLE_SMOKE) ? texSmoke :
                         (p->type == PARTICLE_FLAME) ? texFlame :
                         (p->type == PARTICLE_SNOW)  ? texSnow  :
                         texStar;

            glBindTexture(GL_TEXTURE_2D, tex);

            glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

            // Abilita blending additivo per effetto luce/fumo
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE);

            // Disabilita scrittura nella depth buffer per evitare problemi di trasparenza
            glDepthMask(GL_FALSE);

            float half = size * 0.5f;

            // Recupera la matrice modelview corrente (per billboard)
            float modelview[16];
            glGetFloatv(GL_MODELVIEW_MATRIX, modelview);

            // Estrai i vettori "right" e "up" dalla matrice per orientare la quad verso la camera
            float right[3] = { modelview[0], modelview[4], modelview[8] };
            float up[3]    = { modelview[1], modelview[5], modelview[9] };

            // Calcola i quattro vertici del quadrato orientato verso la camera
            float v1[3] = {
                p->x - half * right[0] - half * up[0],
                p->y - half * right[1] - half * up[1],
                p->z - half * right[2] - half * up[2]
            };
            float v2[3] = {
                p->x + half * right[0] - half * up[0],
                p->y + half * right[1] - half * up[1],
                p->z + half * right[2] - half * up[2]
            };
            float v3[3] = {
                p->x + half * right[0] + half * up[0],
                p->y + half * right[1] + half * up[1],
                p->z + half * right[2] + half * up[2]
            };
            float v4[3] = {
                p->x - half * right[0] + half * up[0],
                p->y - half * right[1] + half * up[1],
                p->z - half * right[2] + half * up[2]
            };

            glColor4f(r, g, b, a);  // Rapplica colore (utile se cambiato precedentemente)

            // Disegna il quadrato texturizzato (quad) per la particella
            glBegin(GL_QUADS);
                glTexCoord2f(0.0f, 0.0f); glVertex3fv(v1);
                glTexCoord2f(1.0f, 0.0f); glVertex3fv(v2);
                glTexCoord2f(1.0f, 1.0f); glVertex3fv(v3);
                glTexCoord2f(0.0f, 1.0f); glVertex3fv(v4);
            glEnd();

            glDisable(GL_TEXTURE_2D);
            glDepthMask(GL_TRUE);  // Riabilita scrittura depth buffer
            break;
        }

        default:
            // Nessuna azione per forme non riconosciute
            break;
    }
}

/**
 * Disegna ricorsivamente una particella e tutti i suoi discendenti (figli, nipoti, ecc.),
 * percorrendo l'intero albero gerarchico delle particelle a partire dalla particella data.
 *
 * parametri in input :
 * p Puntatore alla particella radice da disegnare con i suoi figli
 */
void draw_particle_tree(Particle* p) {
    if (!p || !p->active) return;  // Se la particella è nulla o inattiva, non fare nulla

    draw_particle(p);  // Disegna la particella corrente

    // Ricorsivamente disegna ogni figlio della particella
    for (int i = 0; i < p->child_count; i++) {
        draw_particle_tree(p->children[i]);
    }
}

/**
 * Disegna tutte le particelle attive presenti nell'array globale 'particles'.
 * Scorre tutte le particelle fino a MAX_PARTICLES e chiama draw_particle
 * solo su quelle marcate come attive.
 */
void draw_particles(void) {
    for (int i = 0; i < MAX_PARTICLES; i++) {
        if (particles[i].active)  // Solo particelle attive vengono disegnate
            draw_particle(&particles[i]);
    }
}

//Effetti speciali

/**
 * Lancia un insieme di particelle iniziali per simulare il lancio di un fuoco d'artificio.
 *
 * Per ogni particella:
 * - Genera una posizione iniziale basata su una forma predefinita (es. sfera, cubo).
 * - Applica una rotazione globale per orientare il sistema.
 * - Calcola una velocità direzionale casuale all’interno di un cono stretto verso l’alto.
 * - Imposta colore arancione/giallo con variazioni casuali sul verde.
 * - Inizializza la particella con vita, dimensione e colore variabili.
 * - Segna la particella come emettitore, che potrà generare particelle figlie (scintille).
 *
 * Il numero di particelle lanciate è definito dalla variabile `count`.
 */
void launch_firework(void) {
    int count = 80;  // Numero di particelle da lanciare all'inizio del fuoco d'artificio

    for (int i = 0; i < count; i++) {
        int idx = find_inactive_particle();  // Cerca una particella libera nell'array
        if (idx < 0) break;  // Se non ce ne sono, esci dal ciclo

        // Calcola la posizione iniziale generata da una forma (es. sfera, cubo, ecc.)
        float x, y, z;
        generate_position_from_shape(&x, &y, &z, current_emission_shape);
        apply_system_orientation(&x, &y, &z);  // Applica rotazione/orientamento globale del sistema

        // Genera la direzione del lancio all'interno di un cono stretto (angolo massimo M_PI/6)
        float angle = ((float)rand() / RAND_MAX) * M_PI / 6.0f;
        float theta = ((float)rand() / RAND_MAX) * 2.0f * M_PI;
        float speed = 2.5f + ((float)rand() / RAND_MAX) * 1.5f;  // Velocità casuale da 2.5 a 4.0

        // Componenti della velocità in coordinate cartesiane
        float vx = speed * sinf(angle) * cosf(theta);
        float vy = speed * cosf(angle);
        float vz = speed * sinf(angle) * sinf(theta);

        apply_system_orientation(&vx, &vy, &vz);  // Applica l'orientamento globale alla velocità

        // Colore della particella: arancione/giallo con variazione casuale sul verde
        float r = 1.0f;
        float g = 0.8f + ((float)rand() / RAND_MAX) * 0.2f;
        float b = 0.4f;

        // Inizializza la particella con posizione, velocità, accelerazione (gravità),
        // durata, dimensioni iniziali e finali, colore iniziale e finale, tipo e forma
        init_particle(&particles[idx],
            0.0f, 0.0f, 0.0f,  // posizione iniziale (puoi sostituire con x,y,z se vuoi usarli)
            vx, vy, vz,        // velocità calcolata
            0.0f, gravity * 0.15f, 0.0f,  // accelerazione (gravità verso il basso)
            1.2f + ((float)rand() / RAND_MAX) * 0.5f,  // vita variabile da 1.2 a 1.7 secondi
            0.3f, 0.1f,       // dimensioni iniziale e finale (si rimpicciolisce)
            r, g, b, 1.0f,    // colore iniziale opaco
            r, g, b, 0.0f,    // colore finale trasparente (fade out)
            PARTICLE_FIREWORK, // tipo particella
            0.0f, 0.0f, 0.0f, // origine particella
            SHAPE_POINT);      // forma punto

        // Imposta easing per transizioni di colore e dimensione (migliore smoothness)
        particles[idx].color_easing = EASING_SMOOTHSTEP;
        particles[idx].size_easing = EASING_OUT_CUBIC;
        
        // Questa particella emetterà a sua volta altre particelle (scintille)
        particles[idx].is_emitter = 1;
        particles[idx].emission_rate = 8.0f;   // velocità di emissione figli (8 particelle/sec)
        particles[idx].emission_accumulator = 0.0f;  // reset accumulatore

        // Imposta posizione precedente uguale a quella attuale (per motion blur o linee)
        particles[idx].prev_x = particles[idx].x;
        particles[idx].prev_y = particles[idx].y;
        particles[idx].prev_z = particles[idx].z;
       
    }
}

/**
 * spawn_starfield
 * ----------------
 * Crea un gruppo di particelle che simulano un campo di stelle statiche
 * sparse casualmente in uno spazio tridimensionale.
 *
 * La funzione non prende parametri in input e non restituisce valori.
 * Opera direttamente sull'array globale di particelle 'particles',
 * inizializzando fino a 50 particelle inattive con posizione, dimensione,
 * durata e colore tipici delle stelle (bianco brillante che sfuma nel tempo).
 *
 * Ogni stella ha posizione casuale all'interno di un cubo centrato
 * nell'origine con lato 20 unità, dimensione variabile tra 0.30 e 0.40,
 * durata di vita casuale tra 10 e 20 secondi, e un effetto di dissolvenza
 * (fade-out) verso la trasparenza.
 */
void spawn_starfield(void) {
    int count = 50;  // Numero di stelle da creare

    for (int i = 0; i < count; i++) {
        int idx = find_inactive_particle();  // Trova una particella inattiva da riutilizzare
        if (idx < 0) break;  // Se non ci sono particelle libere, esci dal ciclo

        // Genera posizione casuale all'interno di un cubo centrato nell'origine di lato 20 unità
        float x = ((rand() % 100) / 100.0f - 0.5f) * 20.0f;
        float y = ((rand() % 100) / 100.0f - 0.5f) * 20.0f;
        float z = ((rand() % 100) / 100.0f - 0.5f) * 20.0f;

        // Dimensione casuale della stella tra 0.30 e 0.40
        float size = 0.30f + ((rand() % 100) / 100.0f) * 0.1f;

        // Inizializza la particella: posizione, velocità (ferma), accelerazione (nessuna),
        // durata vita casuale tra 10 e 20 secondi,
        // dimensione iniziale e finale uguali,
        // colore bianco iniziale e trasparente alla fine (fade out),
        // tipo PARTICLE_STAR, forma cerchio morbido per glow effetto
        init_particle(&particles[idx], x, y, z,
                      0.0f, 0.0f, 0.0f,
                      0.0f, 0.0f, 0.0f,
                      10.0f + ((rand() % 100) / 100.0f) * 10.0f,
                      size, size,
                      1.0f, 1.0f, 1.0f, 1.0f,
                      1.0f, 1.0f, 1.0f, 0.0f,
                      PARTICLE_STAR,
                      x, y, z,
                      SHAPE_SOFT_CIRCLE);

        
        // Imposta easing per transizioni di colore e dimensione (migliore smoothness)
        particles[idx].color_easing = EASING_SMOOTHSTEP;
        particles[idx].size_easing = EASING_OUT_CUBIC;

        // La particella può emettere altre particelle (effetto fiamma viva)
        particles[idx].is_emitter = 1;
        particles[idx].emission_rate = 20.0f;
        particles[idx].emission_accumulator = 0.0f;

        // Imposta posizione precedente uguale a quella attuale (utile per motion blur o altre interpolazioni)
        particles[idx].prev_x = particles[idx].x;
        particles[idx].prev_y = particles[idx].y;
        particles[idx].prev_z = particles[idx].z;
        
    }
}

/**
 * launch_three_fireworks
 * ----------------------
 * Lancia simultaneamente tre particelle fuoco d'artificio da posizioni
 * predefinite lungo l'asse X (-2.0, 0.0, 2.0).
 *
 * Ogni particella rappresenta il lancio verticale di un singolo fuoco d'artificio,
 * con velocità iniziale variabile lungo l'asse Y per differenziare l'altezza raggiunta.
 *
 * La funzione non prende input e non restituisce valori, agisce direttamente
 * sull'array globale di particelle, inizializzando fino a tre particelle inattive.
 *
 * Le particelle lanciate hanno una durata di 3 secondi, dimensione che cresce
 * leggermente durante la vita, colore bianco che sfuma a giallo pallido trasparente,
 * e sono di tipo PARTICLE_LAUNCH con forma punto.
 */
void launch_three_fireworks(void) {
    float base_x_positions[3] = {-2.0f, 0.0f, 2.0f};  // Posizioni iniziali lungo l'asse X per i tre fuochi

    for (int i = 0; i < 3; i++) {
        int idx = find_inactive_particle();  // Cerca una particella inattiva da usare
        if (idx < 0) break;  // Se non ci sono particelle libere, esci dal ciclo

        float x = base_x_positions[i];  // Posizione X del lancio
        float y = 0.0f;  // Posizione Y di partenza (terra)
        float z = 0.0f;  // Posizione Z di partenza

        // Velocità verticale iniziale casuale tra 6.0 e 7.0 per variare l’altezza del lancio
        float vy = 6.0f + ((float)rand() / RAND_MAX);

        // Inizializza la particella con:
        // posizione, velocità (solo verso l’alto),
        // accelerazione (gravità),
        // durata di 3 secondi,
        // dimensione iniziale piccola (0.05) e finale leggermente più grande (0.08),
        // colore bianco che sfuma a un giallo pallido trasparente,
        // tipo PARTICLE_LAUNCH e forma punto
        init_particle(&particles[idx],
            x, y, z,
            0.0f, vy, 0.0f,
            0.0f, gravity, 0.0f,
            3.0f,
            0.05f, 0.08f,
            1.0f, 1.0f, 1.0f, 1.0f,
            1.0f, 1.0f, 0.5f, 0.0f,
            PARTICLE_LAUNCH,
            x, y, z,
            SHAPE_POINT);
        
        // Easing per colore e dimensione per un cambio più fluido
        particles[idx].color_easing = EASING_LINEAR;
        particles[idx].size_easing = EASING_IN_CUBIC;
        
        // La particella può emettere altre particelle
        particles[idx].is_emitter = 1;
        particles[idx].emission_rate = 20.0f;
        particles[idx].emission_accumulator = 0.0f;

        // Imposta posizione precedente uguale a quella attuale (utile per motion blur)
        particles[idx].prev_x = x;
        particles[idx].prev_y = y;
        particles[idx].prev_z = z;
    }
}

/**
 * ignite_flame
 * ------------
 * Genera una fiamma composta da numerose particelle con comportamento
 * e colori variabili per simulare un effetto di fiamma naturale e dinamica.
 *
 * Ogni particella viene inizializzata con posizione, velocità, accelerazione,
 * dimensione, colore e durata vita specifici per riprodurre turbolenze e sfumature
 * dal giallo al rosso tipiche di una fiamma.
 *
 * La funzione prende in input:
 *   - flame_angle: (float) angolo preferito della fiamma (attualmente non usato direttamente,
 *                  ma può servire per orientare la fiamma)
 *
 * Non restituisce valori, ma modifica direttamente l'array globale di particelle,
 * generando fino a 700 particelle inattive riattivate e configurate.
 */
void ignite_flame(float flame_angle) {
    int count = 700;  // Numero di particelle della fiamma da generare

    for (int i = 0; i < count; i++) {
        int idx = find_inactive_particle();  // Trova una particella inattiva
        if (idx < 0) break;  // Se non ci sono particelle libere, esci dal ciclo

        // Calcolo posizione iniziale con distribuzione sferica più ampia e più dispersa
        float theta = ((float)rand() / RAND_MAX) * 2.0f * M_PI;  // angolo orizzontale [0, 2π]
        float angle = ((float)rand() / RAND_MAX) * (M_PI / 2.5f);  // angolo verticale più ampio
        float radius = sqrtf((float)rand() / RAND_MAX) * 0.03f;   // raggio distribuito, più ampio
        
        // Coordinate locali nel sistema di riferimento della fiamma
        float local_x = radius * sinf(angle) * cosf(theta);
        float local_y = radius * cosf(angle);
        float local_z = radius * sinf(angle) * sinf(theta);

        // Applica l’orientamento globale del sistema particelle
        float px = local_x, py = local_y, pz = local_z;
        apply_system_orientation(&px, &py, &pz);

        // Direzione preferita della fiamma, leggermente inclinata a destra
        float dx = 0.3f, dy = 0.9f, dz = 0.0f;
        apply_system_orientation(&dx, &dy, &dz);

        // Velocità con turbolenza casuale (rumore per movimento naturale)
        float vx = dx * 3.5f + ((rand() / (float)RAND_MAX) - 0.5f) * 2.2f;
        float vy = dy * 3.5f + ((rand() / (float)RAND_MAX) - 0.5f) * 2.0f;
        float vz = dz * 3.5f + ((rand() / (float)RAND_MAX) - 0.5f) * 2.2f;

        // Valore casuale per variare il colore (da giallo a rosso)
        float h = (rand() / (float)RAND_MAX);  // valore in [0, 1]

        // Calcolo colore base e finale (fade dal giallo all’arancio fino al rosso)
        float r_col = 1.0f;
        float g_col = 0.2f + (1.0f - h) * 0.6f;  // verde variabile tra 0.2 e 0.8
        float b_col = 0.0f;

        // Dimensione iniziale più grande per fiamme più calde, finisce più piccola
        float size_start = 0.20f + (1.0f - h) * 0.15f;
        float size_end = 0.03f;

        // Durata vita della particella variabile per effetto naturale
        float life = 0.9f + ((float)rand() / RAND_MAX) * 1.0f;

        // Inizializza la particella con i parametri calcolati
        init_particle(&particles[idx],
                      px, py, pz,
                      vx, vy, vz,
                      0.0f, gravity * 0.03f, 0.0f,  // accelerazione lenta verso il basso (leggera gravità)
                      life,
                      size_start, size_end,
                      r_col, g_col, b_col, 1.0f,   // colore iniziale
                      r_col, g_col * 0.2f, b_col, 0.0f,  // colore finale (fade out)
                      PARTICLE_FLAME,
                      px, py, pz,
                      SHAPE_SOFT_CIRCLE);

        // Imposta easing per transizioni di colore e dimensione (migliore smoothness)
        particles[idx].color_easing = EASING_SMOOTHSTEP;
        particles[idx].size_easing = EASING_OUT_CUBIC;

        // La particella può emettere altre particelle (effetto fiamma viva)
        particles[idx].is_emitter = 1;
        particles[idx].emission_rate = 20.0f;
        particles[idx].emission_accumulator = 0.0f;

        // Imposta la posizione precedente leggermente indietro rispetto alla velocità per motion blur
        particles[idx].prev_x = px - vx * 0.15f;
        particles[idx].prev_y = py - vy * 0.1f;
        particles[idx].prev_z = pz - vz * 0.1f;
    }
}

/**
 * emit_smoke
 * ----------
 * Genera una serie di particelle che simulano fumo che si solleva da un punto.
 *
 * Ogni particella viene creata con posizione casuale vicina all'origine,
 * velocità leggermente turbolenta e lenta verso l'alto,
 * dimensioni che aumentano nel tempo e un colore grigio chiaro trasparente che sfuma.
 *
 * La funzione non prende input e non restituisce valori,
 * ma modifica direttamente l'array globale di particelle creando fino a 60 particelle di fumo
 * riattivando quelle inattive.
 */
void emit_smoke(void) {
    int count = 60;  // Numero di particelle di fumo da generare in questa chiamata

    for (int i = 0; i < count; i++) {
        int idx = find_inactive_particle();  // Trova una particella inattiva da riutilizzare
        if (idx < 0) break;                   // Se non ce ne sono, esci dal ciclo

        // Genera posizione casuale vicino all'origine (fumo che si alza da un punto)
        float x = ((rand() % 1000) / 1000.0f - 0.5f) * 2.0f;  // X tra -1.0 e +1.0
        float y = ((rand() % 1000) / 1000.0f) * 1.0f;         // Y tra 0 e 1 (sopra il terreno)
        float z = ((rand() % 1000) / 1000.0f - 0.5f) * 2.0f;  // Z tra -1.0 e +1.0

        // Velocità leggermente turbolenta e lenta verso l’alto
        float vx = ((rand() % 1000) / 1000.0f - 0.5f) * 0.05f;  // movimento orizzontale casuale piccolo
        float vy = 0.05f + ((rand() % 1000) / 1000.0f) * 0.05f; // velocità verticale lenta tra 0.05 e 0.1
        float vz = ((rand() % 1000) / 1000.0f - 0.5f) * 0.05f;  // movimento orizzontale casuale piccolo

        // Durata di vita casuale tra 3 e 5 secondi (o frame, a seconda del tuo sistema)
        float life = 3.0f + ((rand() % 1000) / 1000.0f) * 2.0f;

        // Dimensioni iniziale e finale della particella (si espande nel tempo)
        float size_start = 0.4f + ((rand() % 1000) / 1000.0f) * 0.4f;  // tra 0.4 e 0.8
        float size_end = 0.8f + ((rand() % 1000) / 1000.0f) * 0.8f;    // tra 0.8 e 1.6

        // Colore grigio chiaro (quasi bianco) che rimane costante (trasparenza varia)
        float gray_start = 0.9f + ((rand() % 1000) / 1000.0f) * 0.1f;  // tra 0.9 e 1.0
        float gray_end = gray_start;

        // Trasparenza iniziale bassa (fumo molto leggero), poi diventa trasparente
        float alpha_start = 0.05f + ((rand() % 1000) / 1000.0f) * 0.1f;  // tra 0.05 e 0.15
        float alpha_end = 0.0f;

        // Inizializza la particella con tutti i parametri
        init_particle(&particles[idx],
            x, y, z,       // posizione iniziale
            vx, vy, vz,    // velocità
            0.0f, gravity * 0.01f, 0.0f,  // accelerazione (leggera gravità verso il basso)
            life,          // durata vita
            size_start, size_end,  // dimensioni da iniziale a finale
            gray_start, gray_start, gray_start, alpha_start,  // colore iniziale (RGBA)
            gray_end, gray_end, gray_end, alpha_end,          // colore finale (RGBA)
            PARTICLE_SMOKE,  // tipo particella
            x, y, z,         // posizione di origine (per riferimento)
            SHAPE_SOFT_CIRCLE);  // forma da disegnare

        // Easing per colore e dimensione per un cambio più fluido
        particles[idx].color_easing = EASING_LINEAR;
        particles[idx].size_easing = EASING_IN_CUBIC;
        
        // La particella può emettere altre particelle
        particles[idx].is_emitter = 1;
        particles[idx].emission_rate = 20.0f;
        particles[idx].emission_accumulator = 0.0f;

        // Imposta la posizione precedente per motion blur (scia)
        particles[idx].prev_x = x - vx * 0.1f;
        particles[idx].prev_y = y - vy * 0.1f;
        particles[idx].prev_z = z - vz * 0.1f;
    }
}

/**
 * emit_snowfall
 * -------------
 * Genera particelle che simulano una nevicata, con fiocchi di neve che cadono lentamente
 * dall’alto in una vasta area sopra il terreno.
 *
 * Ogni particella viene inizializzata con:
 * - posizione casuale all’interno di un volume sopra il suolo,
 * - velocità di caduta verso il basso con piccoli movimenti orizzontali casuali,
 * - dimensione leggermente variabile,
 * - colore bianco che sfuma lentamente in trasparenza,
 * - durata di vita sufficientemente lunga per simulare la caduta.
 *
 * La funzione non prende input (l’argomento flame_angle non è utilizzato),
 * e non restituisce valori ma modifica direttamente l’array globale delle particelle
 * attivando fino a 80 particelle di tipo PARTICLE_SNOW.
 */
void emit_snowfall(float flame_angle) {
    int count = 80;  // Numero di particelle di neve da generare

    for (int i = 0; i < count; i++) {
        int idx = find_inactive_particle();  // Trova particella inattiva
        if (idx < 0) break;                   // Se nessuna disponibile, esci

        // Posizione iniziale casuale in un’area ampia sopra il terreno
        float x = ((rand() % 1000) / 1000.0f - 0.5f) * 20.0f;  // X tra -10 e +10
        float y = ((rand() % 1000) / 1000.0f) * 10.0f + 5.0f;  // Y tra 5 e 15 (in alto)
        float z = ((rand() % 1000) / 1000.0f - 0.5f) * 20.0f;  // Z tra -10 e +10

        // Velocità leggermente casuale con caduta verso il basso abbastanza veloce
        float vx = ((rand() % 1000) / 1000.0f - 0.5f) * 0.01f;  // Piccolo movimento orizzontale
        float vy = -0.12f - ((rand() % 1000) / 1000.0f) * 0.03f;  // Caduta verticale da -0.12 a -0.15
        float vz = ((rand() % 1000) / 1000.0f - 0.5f) * 0.01f;  // Piccolo movimento orizzontale

        // Dimensione particelle neve, leggermente variabile
        float size = 0.30f + ((rand() % 100) / 100.0f) * 0.1f;  // Da 0.30 a 0.40

        // Inizializza particella neve
        init_particle(&particles[idx],
            x, y, z,     // posizione
            vx, vy, vz,  // velocità
            0.0f, 0.0f, 0.0f,  // accelerazione (nessuna)
            35.0f,       // durata vita (modifica questo valore per far cadere la neve più lentamente o velocemente)
            size, size,  // dimensione iniziale e finale (costante)
            1.0f, 1.0f, 1.0f, 1.0f,  // colore bianco opaco all’inizio (RGBA)
            1.0f, 1.0f, 1.0f, 0.0f,  // colore bianco trasparente alla fine (fade out)
            PARTICLE_SNOW,  // tipo particella
            x, y, z,       // posizione origine per riferimento
            SHAPE_SOFT_CIRCLE);  // forma della particella

        // Easing lineare per colore e dimensione
        particles[idx].color_easing = EASING_LINEAR;
        particles[idx].size_easing = EASING_LINEAR;
        
        // La particella può emettere altre particelle
        particles[idx].is_emitter = 1;
        particles[idx].emission_rate = 20.0f;
        particles[idx].emission_accumulator = 0.0f;

        // Posizione precedente per motion blur/scia (piccola frazione della velocità)
        particles[idx].prev_x = x - vx * 0.05f;
        particles[idx].prev_y = y - vy * 0.05f;
        particles[idx].prev_z = z - vz * 0.05f;
    }
}


//Input e controllo

/**
 * key_callback
 * -------------
 * Gestisce gli input da tastiera, eseguendo azioni diverse in base al tasto premuto. La funzione
 * è chiamata ogni volta che una pressione o un rilascio di tasto viene rilevato dalla finestra.
 *
 * Funzionalità principali:
 * - Attivazione/spegnimento di effetti come fuochi artificiali, fiamma, neve, fumo.
 * - Modifica delle proprietà del sistema, come inclinazione, rotazione, scala, e posizione della camera.
 * - Cambio della forma di emissione delle particelle.
 * - Pausa del sistema o chiusura della finestra.
 *
 * parametri in input :
 * - `GLFWwindow* w`: Puntatore alla finestra su cui viene ricevuto l'input.
 * - `int key`: Codice del tasto premuto.
 * - `int sc`: Codice scanc (non utilizzato in questo caso).
 * - `int action`: Tipo di azione eseguita (GLFW_PRESS o GLFW_REPEAT).
 * - `int mods`: Modificatori della tastiera (come SHIFT, ALT, ecc., non utilizzato in questo caso).
 *
 * Azioni eseguite in base ai tasti premuti:
 * - **Q**: Toggle accensione/spegnimento fuochi artificiali (`launch_active`).
 * - **E**: Toggle attivazione fiamma (`flame_active`).
 * - **Y**: Genera stelle di sfondo (`spawn_starfield`).
 * - **S**: Lancia tre fuochi artificiali insieme (`launch_three_fireworks`).
 * - **T**: Emetti fumo (`emit_smoke`).
 * - **N**: Toggle neve attiva (`snow_active`).
 * - **L**: Spegne fuochi e fiamma, pulisce tutte le particelle (`clear_all_particles`).
 * - **I/K**: Aumenta/diminuisce l'inclinazione (pitch) del sistema (`system_pitch_deg`).
 * - **J/B**: Ruota a sinistra/destra (yaw) del sistema (`system_yaw_deg`).
 * - **X/V/C/Z**: Cambia la forma di emissione delle particelle tra punto, cerchio, rettangolo e sfera.
 * - **Space**: Pausa o riprendi il sistema (`paused`).
 * - **Escape**: Chiude la finestra e termina il programma.
 * - **A/D**: Movimento orizzontale della camera (`cam_pan_x`).
 * - **Freccia sinistra/destra**: Rotazione della camera sull'asse X (`cam_angle_x`).
 * - **Freccia su/giù**: Rotazione della camera sull'asse Y con limitazioni di angolo per evitare rotazioni eccessive (`cam_angle_y`).
 * - Premere `[` : Riduce la scala scena sull'asse X (`scene_scale_x`).
 * - Premere `]`: Aumenta la scala scena sull'asse X (`scene_scale_x`).
 * - **W/P**: Riduce/aumenta la scala scena sull'asse Y (`scene_scale_y`).
 * - **M/U**: Zoom della camera indietro/avanti, modificando la distanza dalla scena (`cam_dist`).
 *
 * La funzione non restituisce alcun valore, ma modifica variabili globali che influenzano il comportamento del sistema di particelle e della scena.
 */
void key_callback(GLFWwindow* w, int key, int sc, int action, int mods) {
    // Ignora se non è pressione o ripetizione
    if (action != GLFW_PRESS && action != GLFW_REPEAT) return;

    switch (key) {
        case GLFW_KEY_Q:
            // Toggle accensione/spegnimento lancio fuochi artificiali
            launch_active = !launch_active;
            break;

        case GLFW_KEY_E:
            // Toggle fiamma attiva
            flame_active = !flame_active;
            break;

        case GLFW_KEY_Y:
            // Genera stelle di sfondo
            spawn_starfield();
            break;

        case GLFW_KEY_S:
            // Lancia tre fuochi artificiali insieme
            launch_three_fireworks();
            break;

        case GLFW_KEY_T:
            // Emetti fumo
            emit_smoke();
            break;

        case GLFW_KEY_N:
            // Toggle neve attiva
            snow_active = !snow_active;
            break;

        case GLFW_KEY_L:
            // Spegni fuochi e fiamma e pulisci tutte le particelle
            launch_active = 0;
            flame_active = 0;
            clear_all_particles();
            break;

        case GLFW_KEY_I:
            // Inclinazione verso l’alto (aumenta pitch)
            system_pitch_deg += 5.0f;
            printf("Pitch: %.1f°, Yaw: %.1f°\n", system_pitch_deg, system_yaw_deg);
            break;

        case GLFW_KEY_K:
            // Inclinazione verso il basso (diminuisci pitch)
            system_pitch_deg -= 5.0f;
            printf("Pitch: %.1f°, Yaw: %.1f°\n", system_pitch_deg, system_yaw_deg);
            break;

        case GLFW_KEY_J:
            // Rotazione a sinistra (diminuisci yaw)
            system_yaw_deg -= 5.0f;
            printf("Pitch: %.1f°, Yaw: %.1f°\n", system_pitch_deg, system_yaw_deg);
            break;

        case GLFW_KEY_B:
            // Rotazione a destra (aumenta yaw)
            system_yaw_deg += 5.0f;
            printf("Pitch: %.1f°, Yaw: %.1f°\n", system_pitch_deg, system_yaw_deg);
            break;

        // Cambia forma emissione particelle
        case GLFW_KEY_X:
            current_emission_shape = SHAPE_POINT;
            printf("Emission shape changed to %d\n", current_emission_shape);
            break;
        case GLFW_KEY_V:
            current_emission_shape = SHAPE_CIRCLE_XY;
            printf("Emission shape changed to %d\n", current_emission_shape);
            break;
        case GLFW_KEY_C:
            current_emission_shape = SHAPE_RECT_XY;
            printf("Emission shape changed to %d\n", current_emission_shape);
            break;
        case GLFW_KEY_Z:
            current_emission_shape = SHAPE_SPHERE;
            printf("Emission shape changed to %d\n", current_emission_shape);
            break;

        case GLFW_KEY_SPACE:
            // Pausa o riprendi il sistema
            paused = !paused;
            break;

        case GLFW_KEY_ESCAPE:
            // Chiudi la finestra e termina il programma
            glfwSetWindowShouldClose(w, GLFW_TRUE);
            break;

        // Movimento camera in orizzontale
        case GLFW_KEY_A:
            cam_pan_x -= 0.3f;
            break;
        case GLFW_KEY_D:
            cam_pan_x += 0.3f;
            break;

        // Rotazione camera asse X (orizzontale)
        case GLFW_KEY_LEFT:
            cam_angle_x -= 3.0f;
            break;
        case GLFW_KEY_RIGHT:
            cam_angle_x += 3.0f;
            break;

        // Rotazione camera asse Y (verticale) con clamp per evitare rotazione eccessiva
        case GLFW_KEY_UP:
            cam_angle_y -= 3.0f;
            cam_angle_y = clamp(cam_angle_y, -89.0f, 89.0f);
            break;
        case GLFW_KEY_DOWN:
            cam_angle_y += 3.0f;
            cam_angle_y = clamp(cam_angle_y, -89.0f, 89.0f);
            break;

        // Modifica scala scena asse X
        case GLFW_KEY_LEFT_BRACKET:  // '['
            scene_scale_x -= 0.05f;
            if(scene_scale_x < 0.1f) scene_scale_x = 0.1f;  // limite minimo
            break;
        case GLFW_KEY_RIGHT_BRACKET:  // ']'
            scene_scale_x += 0.05f;
            break;

        // Modifica scala scena asse Y
        case GLFW_KEY_W:  // qui usato come '{'
            scene_scale_y -= 0.05f;
            if(scene_scale_y < 0.1f) scene_scale_y = 0.1f;  // limite minimo
            break;
        case GLFW_KEY_P:  // qui usato come '}'
            scene_scale_y += 0.05f;
            break;

        // Zoom camera indietro (distanza aumenta)
        case GLFW_KEY_M:  // tasto '-'
            cam_dist += 1.0f;
            if (cam_dist > 100.0f) cam_dist = 100.0f;
            break;

        // Zoom camera avanti (distanza diminuisce)
        case GLFW_KEY_U:  // tasto '='
            cam_dist -= 1.0f;
            if (cam_dist < 2.0f) cam_dist = 2.0f;
            break;
    }
}

/**
 * mouse_button_callback
 * ---------------------
 * Gestisce gli eventi relativi ai pulsanti del mouse, come la pressione e il rilascio del tasto sinistro o destro del mouse.
 * La funzione viene chiamata ogni volta che un pulsante del mouse viene premuto o rilasciato.
 *
 * Funzionalità principali:
 * - Aggiorna lo stato del tasto sinistro e destro del mouse (`left_mouse_down`, `right_mouse_down`).
 * - Tiene traccia della posizione corrente del cursore del mouse (`last_mouse_x`, `last_mouse_y`).
 *
 * parametri in input :
 * - `GLFWwindow* window`: Puntatore alla finestra su cui l'evento del mouse è stato generato.
 * - `int button`: Identificatore del pulsante premuto (sinistro o destro).
 * - `int action`: Azione eseguita sul pulsante (GLFW_PRESS per pressione, GLFW_RELEASE per rilascio).
 * - `int mods`: Modificatori della tastiera (come SHIFT o ALT, ma non utilizzato in questo caso).
 *
 * Azioni eseguite:
 * - Se il tasto sinistro del mouse viene premuto o rilasciato (`GLFW_MOUSE_BUTTON_LEFT`), la variabile `left_mouse_down` viene aggiornata.
 * - Se il tasto destro del mouse viene premuto o rilasciato (`GLFW_MOUSE_BUTTON_RIGHT`), la variabile `right_mouse_down` viene aggiornata.
 * - La funzione aggiorna sempre la posizione del cursore del mouse con `glfwGetCursorPos` per tenerne traccia.
 *
 * Variabili globali modificate:
 * - `left_mouse_down`: Indica se il tasto sinistro del mouse è attualmente premuto o meno.
 * - `right_mouse_down`: Indica se il tasto destro del mouse è attualmente premuto o meno.
 * - `last_mouse_x`, `last_mouse_y`: Posizione corrente del cursore del mouse (aggiornate ad ogni evento).
 *
 * La funzione non restituisce alcun valore, ma modifica lo stato di diverse variabili globali che possono essere usate
 * in altre parti del programma, ad esempio per interazioni con la scena o la telecamera.
 */
void mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
    // Se si preme o rilascia il tasto sinistro del mouse
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        // Aggiorna lo stato del tasto sinistro (premuto = true, rilasciato = false)
        left_mouse_down = (action == GLFW_PRESS);
    }

    // Se si preme o rilascia il tasto destro del mouse
    if (button == GLFW_MOUSE_BUTTON_RIGHT) {
        // Aggiorna lo stato del tasto destro (premuto = true, rilasciato = false)
        right_mouse_down = (action == GLFW_PRESS);
    }

    // Aggiorna la posizione corrente del cursore del mouse
    // Questo serve per tenere traccia della posizione iniziale o per calcolare spostamenti successivi
    glfwGetCursorPos(window, &last_mouse_x, &last_mouse_y);
}

/**
 * cursor_position_callback
 * -------------------------
 * Gestisce gli eventi relativi alla posizione del cursore del mouse e interagisce con la scena.
 * La funzione viene chiamata ogni volta che il cursore del mouse si sposta all'interno della finestra.
 *
 * Funzionalità principali:
 * - Calcola lo spostamento del mouse rispetto alla posizione precedente.
 * - Controlla se uno dei tasti Shift è premuto e cambia il comportamento dell'interazione di conseguenza.
 * - Modifica la posizione della telecamera o la scala della scena in base allo spostamento del mouse e ai tasti premuti.
 *
 * parametri in input :
 * - `GLFWwindow* window`: Puntatore alla finestra in cui l'evento del cursore è stato generato.
 * - `double xpos`: Posizione X attuale del cursore del mouse nella finestra.
 * - `double ypos`: Posizione Y attuale del cursore del mouse nella finestra.
 *
 * Azioni eseguite:
 * - Calcola lo spostamento del mouse in entrambe le direzioni (orizzontale e verticale) rispetto alla posizione precedente (`last_mouse_x`, `last_mouse_y`).
 * - Se il tasto sinistro del mouse è premuto (`left_mouse_down`):
 *     - Se il tasto Shift è premuto, modifica la scala della scena (zoom) in base allo spostamento del mouse.
 *     - Altrimenti, ruota la camera in base allo spostamento del mouse, regolando gli angoli `cam_angle_x` e `cam_angle_y`.
 * - Se il tasto destro del mouse è premuto (`right_mouse_down`), muove la telecamera orizzontalmente o verticalmente (pan).
 * - La posizione precedente del cursore viene aggiornata per il prossimo frame.
 *
 * Variabili globali modificate:
 * - `last_mouse_x`, `last_mouse_y`: Aggiornate alla posizione corrente del cursore.
 * - `cam_angle_x`, `cam_angle_y`: Angoli di rotazione della camera.
 * - `cam_pan_x`, `cam_pan_y`: Posizione della telecamera per il movimento laterale (pan).
 * - `scene_scale_x`, `scene_scale_y`: Scala della scena in entrambe le direzioni.
 *
 * La funzione non restituisce alcun valore, ma modifica lo stato di variabili globali che influenzano
 * l'aspetto della scena o il comportamento della telecamera.
 */
void cursor_position_callback(GLFWwindow* window, double xpos, double ypos) {
    // Calcola lo spostamento del mouse rispetto all'ultima posizione
    double dx = xpos - last_mouse_x;
    double dy = ypos - last_mouse_y;

    // Controlla se uno dei tasti Shift è premuto
    int shift_held = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) ||
                     (glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    // Se il tasto sinistro del mouse è premuto
    if (left_mouse_down) {
        if (shift_held) {
            // Con Shift: modifica la scala della scena in base allo spostamento del mouse
            scene_scale_x += dx * 0.005f;
            scene_scale_y -= dy * 0.005f;
            
            // Limita la scala per non diventare troppo piccola
            if(scene_scale_x < 0.1f) scene_scale_x = 0.1f;
            if(scene_scale_y < 0.1f) scene_scale_y = 0.1f;
        } else {
            // Senza Shift: ruota la camera in base allo spostamento del mouse
            cam_angle_x += (float)dx * 0.3f;
            cam_angle_y += (float)dy * 0.3f;
            // Limita l'angolo verticale per evitare rotazioni eccessive (come capovolgimenti)
            cam_angle_y = clamp(cam_angle_y, -89.0f, 89.0f);
            
            printf("cam_angle_x: %.2f, cam_angle_y: %.2f\n", cam_angle_x, cam_angle_y);
        }
    }
    // Se il tasto destro del mouse è premuto
    else if (right_mouse_down) {
        // Muovi la camera lateralmente (pan) in base allo spostamento del mouse
        cam_pan_x += (float)dx * 0.01f;
        cam_pan_y -= (float)dy * 0.01f;
    }

    // Aggiorna la posizione ultima del mouse per il prossimo frame
    last_mouse_x = xpos;
    last_mouse_y = ypos;
}

/**
 * scroll_callback
 * ---------------
 * Gestisce l'evento di scroll del mouse per il controllo dello zoom della telecamera.
 * La funzione viene chiamata ogni volta che l'utente utilizza la rotellina del mouse per fare zoom in o zoom out.
 *
 * Funzionalità principali:
 * - Modifica la distanza della telecamera dal punto di interesse (zoom in/out).
 * - Limita la distanza minima e massima della telecamera per evitare effetti indesiderati (telecamera troppo vicina o troppo lontana).
 *
 * parametri in input :
 * - `GLFWwindow* window`: Puntatore alla finestra in cui l'evento di scroll è stato generato.
 * - `double xoffset`: Spostamento orizzontale della rotellina (non utilizzato in questo caso).
 * - `double yoffset`: Spostamento verticale della rotellina (utilizzato per zoomare in/out).
 *
 * Azioni eseguite:
 * - Modifica la distanza della telecamera (variabile `cam_dist`) in base allo spostamento verticale della rotellina.
 * - Se l'utente scrolla verso l'alto, la distanza della telecamera si riduce (zoom in), avvicinando la scena.
 * - Se l'utente scrolla verso il basso, la distanza aumenta (zoom out), allontanando la scena.
 * - La distanza della telecamera è limitata tra un valore minimo (`2.0f`) e un valore massimo (`50.0f`) per evitare di entrare troppo nella scena o di allontanarsi troppo da essa.
 *
 * Variabili globali modificate:
 * - `cam_dist`: La distanza attuale della telecamera dal punto di interesse. La funzione la aggiorna in base al valore di `yoffset`.
 *
 * La funzione non restituisce alcun valore, ma modifica la variabile `cam_dist` che influisce sulla visualizzazione della scena.
 */
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
    // Zoom in/out modificando la distanza della camera dal punto di interesse
    cam_dist -= (float)yoffset;

    // Limita la distanza minima della camera per evitare di entrare dentro la scena
    if (cam_dist < 2.0f) cam_dist = 2.0f;

    // Limita la distanza massima della camera per evitare di allontanarsi troppo
    if (cam_dist > 50.0f) cam_dist = 50.0f;
}


//Utility di sorting

/**
 * compare_particle_distance
 * --------------------------
 * Funzione di confronto utilizzata per ordinare un array di particelle in base alla loro distanza dalla telecamera.
 * La funzione viene tipicamente usata con `qsort` per ordinare le particelle dalla più lontana alla più vicina,
 * in modo da garantire un corretto ordine di rendering (ad esempio per la gestione della trasparenza).
 *
 * parametri in input :
 * - `a`: Un puntatore generico a un elemento dell'array, che rappresenta un puntatore a una particella (`Particle*`).
 * - `b`: Un altro puntatore generico a un elemento dell'array, che rappresenta anch'esso un puntatore a una particella (`Particle*`).
 *
 * Entrambi i puntatori (`a` e `b`) devono essere castati a puntatori a particelle, poiché `qsort` passa i dati come `void*`.
 *
 * ritorna in output:
 * - Restituisce:
 *   - `1` se la particella `pb` è più lontana della particella `pa` dalla telecamera.
 *   - `-1` se la particella `pb` è più vicina della particella `pa` alla telecamera.
 *   - `0` se entrambe le particelle sono alla stessa distanza dalla telecamera.
 *
 * La funzione viene usata in combinazione con `qsort` per ordinare un array di particelle, ad esempio per ordinare le particelle
 * in base alla loro distanza dalla telecamera per il rendering.
 */
int compare_particle_distance(const void* a, const void* b) {
    // cast a doppio puntatore a Particle (vettore di puntatori)
    const Particle* pa = *(const Particle**)a;
    const Particle* pb = *(const Particle**)b;

    // confronto tra distanze dalla camera
    // restituisce 1 se pb è più lontana di pa,
    // -1 se pb è più vicina, 0 se uguali
    return (pb->camera_dist > pa->camera_dist) - (pb->camera_dist < pa->camera_dist);
}


/* ************************************************************************ */
// MAIN

int main(void) {
    // Inizializza il seme per la generazione casuale
    srand((unsigned int)time(NULL));

    //Inizializza GLFW (finestra e contesto OpenGL)
    if (!glfwInit()) return -1;

    //Crea una finestra 800x800
    GLFWwindow* window = glfwCreateWindow(800, 800, "Reeves Particle System", NULL, NULL);
    if (!window) {
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);

    //Attiva VSync per sincronizzazione con il refresh rate
    glfwSwapInterval(1);

    //Inizializza GLEW (necessario per funzioni OpenGL estese)
    glewInit();

    //Carica le texture da file (fuoco, fumo, stelle, neve)
    loadTextures();
    printf("texFlame: %u, texSmoke: %u, texStar: %u\n", texFlame, texSmoke, texStar);

    //Registra i callback per input da tastiera e mouse
    glfwSetKeyCallback(window, key_callback);
    glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwSetCursorPosCallback(window, cursor_position_callback);
    glfwSetScrollCallback(window, scroll_callback);

    //Imposta blending additivo e smoothing per linee e punti
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE);
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_POINT_SMOOTH);
    glPointSize(10);

    //Alloca memoria per il sistema particellare
    allocate_particle_system(2000);

    //Variabili per gestione del tempo e frame
    double lastTime = glfwGetTime();
    int global_frame = 0;
    f0 = 0;

    //CICLO PRINCIPALE
    while (!glfwWindowShouldClose(window)) {
        //Calcola quante particelle emettere in questo frame
        int n = compute_particles_this_frame(global_frame);
        if (launch_active) {
            for (int i = 0; i < n; i++) {
                launch_firework();  // Emissione fuochi d'artificio
            }
        }

        //Ottiene dimensioni della finestra
        int width, height;
        glfwGetFramebufferSize(window, &width, &height);

        //Calcola delta time (tempo trascorso)
        double currentTime = glfwGetTime();
        float dt = (float)(currentTime - lastTime);

        //Controllo framerate (target: 60 FPS)
        if (dt < FRAME_DURATION) {
            while (glfwGetTime() - lastTime < FRAME_DURATION) {
                // attesa attiva
            }
            currentTime = glfwGetTime();
            dt = (float)(currentTime - lastTime);
        }
        lastTime = currentTime;

        //Setup della proiezione e camera
        float aspect = (float)width / (float)height;
        glViewport(0, 0, width, height);
        glClearColor(0, 0, 0, 1);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glEnable(GL_DEPTH_TEST);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(45.0f, aspect, 0.1f, 1000.0f);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        // Calcolo posizione camera da distanza e angoli
        float camX = cam_dist * sinf(cam_angle_x * M_PI / 180.0f) * cosf(cam_angle_y * M_PI / 180.0f);
        float camY = cam_dist * sinf(cam_angle_y * M_PI / 180.0f);
        float camZ = cam_dist * cosf(cam_angle_x * M_PI / 180.0f) * cosf(cam_angle_y * M_PI / 180.0f);

        // Aggiunge lo spostamento (pan) sul target
        float targetX = cam_pan_x;
        float targetY = cam_pan_y;
        float targetZ = 0.0f;

        // Camera look-at (posizione camera, centro, vettore up)
        gluLookAt(camX + targetX, camY + targetY, camZ,
                  targetX, targetY, targetZ,
                  0.0f, 1.0f, 0.0f);  // Up = Y positivo
        glScalef(scene_scale_x, scene_scale_y, 1.0f);

        //AGGIORNAMENTO PARTICELLE
        if (!paused) {
            if (flame_active) ignite_flame(20.0f);
            if (snow_active) emit_snowfall(20.0f);

            // Aggiorna ricorsivamente solo le particelle radice
            for (int i = 0; i < particle_capacity; i++) {
                if (particles[i].active && particles[i].parent == NULL) {
                    update_particle_tree(&particles[i], dt);
                }
            }
        }

        //Disegno particelle
        glDepthMask(GL_FALSE);  // Evita scrittura nello Z-buffer per trasparenze

        for (int i = 0; i < particle_capacity; i++) {
            if (particles[i].active && particles[i].parent == NULL) {
                draw_particle_tree(&particles[i]);
            }
        }

        glDepthMask(GL_TRUE);  // Ripristina scrittura Z-buffer per altri oggetti

        //Swap dei buffer e polling eventi
        glfwSwapBuffers(window);
        glfwPollEvents();
        global_frame++;
    }

    //Cleanup finale
    free_particle_system();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
