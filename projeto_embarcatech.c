#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "inc/ssd1306.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "ws2818b.pio.h"

//--------------------------------------------------------------
// DEFINIÇÕES DO LABIRINTO
#define CELL_FREE    0
#define CELL_WALL    1
#define CELL_GOAL    3

int current_g_for_display = 0;
int current_h_for_display = 0;


#define MAZE_WIDTH 5
#define MAZE_HEIGHT 5
#define MAX_QUEUE 50
#define MAX_HISTORY 50  // Máximo de movimentos para voltar

typedef struct {
    int x;
    int y;
    int g;  // Custo acumulado
    int h;  // Heurística
    int f;  // g + h
} AStarNode;

typedef struct {
    int x;
    int y;
    AStarNode astar_queue[MAX_QUEUE]; // Estado da fila do A*
    int astar_front;
    int astar_rear;
} HistoryEntry;

HistoryEntry history[MAX_HISTORY]; // Histórico de estados
int current_step = 0;              // Passo atual no histórico

typedef struct {
    int x;
    int y;
    int h;
} MoveInfo;

MoveInfo best_move;
MoveInfo worse_move;

// Estados do sistema
enum AppState { 
    STATE_INTRO, 
    STATE_RUNNING, 
    STATE_GOAL_REACHED 
};
enum Algorithm {
    ALG_GREEDY,
    ALG_ASTAR,
    ALG_TOTAL
};

enum AppState app_state = STATE_INTRO;
enum Algorithm current_algorithm = ALG_GREEDY;

AStarNode astar_queue[MAX_QUEUE];
int astar_front = -1, astar_rear = -1;

// Labirinto 5x5
int maze[MAZE_HEIGHT][MAZE_WIDTH] = {
    {0, 0, 1, 1, 0},  // Posição inicial (0,0)
    {0, 1, 0, 1, 0},
    {0, 0, 0, 0, 0},
    {0, 1, 0, 1, 0},
    {0, 0, 0, 1, 3}   // Meta em (4,4)  
};

// Posição atual e meta
int current_x = 0;
int current_y = 0;
int goal_x = 4;
int goal_y = 4;

//--------------------------------------------------------------
// DEFINIÇÕES DE HARDWARE
#define LED_RED_PIN         13
#define LED_GREEN_PIN       11
#define LED_BLUE_PIN        12
#define BUTTON_A_PIN         5
#define BUTTON_B_PIN         6
#define JOYSTICK_BUTTON_PIN 22
#define JOYSTICK_X_PIN      27
#define JOYSTICK_Y_PIN      26
#define BUZZER_PIN          21
#define WS2812_PIN           7

// OLED
#define I2C_SDA_PIN         14
#define I2C_SCL_PIN         15
i2c_inst_t *i2c = i2c1;

// Matriz de LEDs
#define LED_COUNT 25
struct pixel_t { uint8_t G, R, B; };
typedef struct pixel_t pixel_t;
typedef pixel_t npLED_t;
npLED_t leds[LED_COUNT];
PIO np_pio;
uint sm;

void updateLEDMatrix(int current_h, int best, int worse);
int get_best_neighbor(int x, int y, MoveInfo *best, MoveInfo *worse);

//--------------------------------------------------------------
// FUNÇÕES DA MATRIZ DE LEDS
void npInit(uint pin) {
    uint offset = pio_add_program(pio0, &ws2818b_program);
    np_pio = pio0;
    sm = pio_claim_unused_sm(np_pio, true);
    ws2818b_program_init(np_pio, sm, offset, pin, 800000.f);
    for(uint i = 0; i < LED_COUNT; i++) leds[i] = (npLED_t){0,0,0};
}

void npSetLED(uint index, uint8_t r, uint8_t g, uint8_t b) {
    if(index < LED_COUNT) {
        leds[index].R = r;
        leds[index].G = g;
        leds[index].B = b;
    }
}

void npWrite() {
    for(uint i = 0; i < LED_COUNT; i++) {
        pio_sm_put_blocking(np_pio, sm, leds[i].G);
        pio_sm_put_blocking(np_pio, sm, leds[i].R);
        pio_sm_put_blocking(np_pio, sm, leds[i].B);
    }
    sleep_us(100);
}
void npClear() {
    for (uint i = 0; i < LED_COUNT; i++) {
        npSetLED(i, 0, 0, 0);
    }
}

void clear_led_matrix() {
    npClear();
    npWrite();
}

//--------------------------------------------------------------
// FUNÇÕES DE INTERFACE EDUCATIVA (atualizadas)
void show_intro_screens() {
    struct render_area frame_area = {0, ssd1306_width-1, 0, ssd1306_n_pages-1};
    calculate_render_area_buffer_length(&frame_area);
    uint8_t ssd[ssd1306_buffer_length];

    // Tela 1: Explicação dos Algoritmos
    memset(ssd, 0, ssd1306_buffer_length);
    ssd1306_draw_string(ssd, 0, 0,  "Algoritmos:");
    ssd1306_draw_string(ssd, 0, 10, "GULOSO");
    ssd1306_draw_string(ssd, 0, 20, "A ESTRELA");
    ssd1306_draw_string(ssd, 0, 40, "Joystick para");
    ssd1306_draw_string(ssd, 0, 50, "trocar algoritmo");
    render_on_display(ssd, &frame_area);
    while(gpio_get(BUTTON_A_PIN));

    sleep_ms(200);

    // Tela 2: Controles
    memset(ssd, 0, ssd1306_buffer_length);
    ssd1306_draw_string(ssd, 0, 0,  "Controles:");
    ssd1306_draw_string(ssd, 0, 10, "A: Proximo passo");
    ssd1306_draw_string(ssd, 0, 20, "B: Volta passo");
    ssd1306_draw_string(ssd, 0, 30, "Botao Joystick:");
    ssd1306_draw_string(ssd, 0, 40, "Troca algoritmo");
    render_on_display(ssd, &frame_area);
    while(gpio_get(BUTTON_A_PIN));
}

//--------------------------------------------------------------
// FUNÇÕES DO OLED (atualizadas)
void update_algorithm_display() {
    struct render_area frame_area = {0, ssd1306_width-1, 0, ssd1306_n_pages-1};
    uint8_t ssd[ssd1306_buffer_length];
    memset(ssd, 0, ssd1306_buffer_length);
    
    const char* alg;
    switch(current_algorithm) {
        case ALG_GREEDY: alg = "GULOSO"; break;
        case ALG_ASTAR:  alg = "A STAR";  break;
        default:         alg = "DESCONHECIDO";
    }
    
    ssd1306_draw_string(ssd, 0, 0, "Modo Atual:");
    ssd1306_draw_string(ssd, 0, 20, (char*)alg);
    render_on_display(ssd, &frame_area);
}

//--------------------------------------------------------------
// FUNÇÕES MODIFICADAS PARA HISTÓRICO
void save_position(int x, int y) {
    if (current_step < MAX_HISTORY - 1) {
        history[current_step].x = x;
        history[current_step].y = y;

        // Salva o estado da fila do A* somente para movimentos efetivos
        if (current_algorithm == ALG_ASTAR) {
            history[current_step].astar_front = astar_front;
            history[current_step].astar_rear = astar_rear;
            for (int i = 0; i <= astar_rear; i++) {
                history[current_step].astar_queue[i] = astar_queue[i];
            }
        }
        current_step++;
    }
}

void print_history(void) {
    printf("\n=== Histórico de Movimentos ===\n");
    for (int i = 0; i < current_step; i++) {
        printf("Passo %d: Posição = (%d, %d)", i, history[i].x, history[i].y);
        if (current_algorithm == ALG_ASTAR) {
            printf(" | Fila A*: front = %d, rear = %d", history[i].astar_front, history[i].astar_rear);
            printf(" ->");
            for (int j = history[i].astar_front + 1; j <= history[i].astar_rear; j++) {
                printf(" (%d,%d)[f=%d]", history[i].astar_queue[j].x, history[i].astar_queue[j].y, history[i].astar_queue[j].f);
            }
        }
        printf("\n");
    }
    printf("=== Fim do Histórico ===\n\n");
}

void load_previous_position() {
    if (current_step > 0) {
        current_step--;
        current_x = history[current_step].x;
        current_y = history[current_step].y;
        if (current_algorithm == ALG_ASTAR) {
            print_history();
            astar_front = history[current_step].astar_front;
            astar_rear = history[current_step].astar_rear;
            for (int i = 0; i <= astar_rear; i++) {
                astar_queue[i] = history[current_step].astar_queue[i];
            }
        }
    }
}

// Função para reinicializar a fila do A* sem zerar o histórico
void reinit_astar_frontier(void) {
    astar_front = -1;
    astar_rear = -1;
    int h = manhattan(current_x, current_y, goal_x, goal_y);
    AStarNode start = {current_x, current_y, 0, h, h};
    astar_queue[++astar_rear] = start;
    // Opcional: você pode salvar o movimento atual, se desejar.
    // save_position(current_x, current_y);
}

// FUNÇÃO DO BUZZER
void play_buzzer(uint32_t frequency, uint32_t duration_ms) {
    uint32_t period_us = 1000000 / frequency;
    uint32_t end_time = to_ms_since_boot(get_absolute_time()) + duration_ms;
    
    while (to_ms_since_boot(get_absolute_time()) < end_time) {
        gpio_put(BUZZER_PIN, 1);
        sleep_us(period_us / 2);
        gpio_put(BUZZER_PIN, 0);
        sleep_us(period_us / 2);
    }
}

//--------------------------------------------------------------
// FUNÇÕES DO LABIRINTO
void get_astar_min_max_f(int *min_f, int *max_f) {
    *min_f = 9999;
    *max_f = -1;
    
    for (int i = astar_front + 1; i <= astar_rear; i++) {
        if (astar_queue[i].f < *min_f) *min_f = astar_queue[i].f;
        if (astar_queue[i].f > *max_f) *max_f = astar_queue[i].f;
    }
}

void updateOLEDDisplay(int current_h, int best, int worse) {
    struct render_area frame_area = {0, ssd1306_width-1, 0, ssd1306_n_pages-1};
    calculate_render_area_buffer_length(&frame_area);
    
    uint8_t ssd[ssd1306_buffer_length];
    memset(ssd, 0, ssd1306_buffer_length);

    char buf[50];

    // Se estamos no algoritmo A*, mostrar "H: h + g"
    if (current_algorithm == ALG_ASTAR) {
        snprintf(buf, sizeof(buf), "H: %d + %d", current_h_for_display, current_g_for_display);
        ssd1306_draw_string(ssd, 0, 0, buf);
    } 
    else {
        // Caso seja Greedy, exibe a heurística atual normalmente
        snprintf(buf, sizeof(buf), "Atual H: %d", current_h);
        ssd1306_draw_string(ssd, 0, 0, buf);
    }

    // Exibe melhor e pior H (pode manter como antes)
    snprintf(buf, sizeof(buf), "Melhor H: %d", best);
    ssd1306_draw_string(ssd, 0, 20, buf);

    snprintf(buf, sizeof(buf), "Pior H: %d", worse);
    ssd1306_draw_string(ssd, 0, 30, buf);

    // Nome do algoritmo
    const char* algorithm_name;
    switch(current_algorithm) {
        case ALG_GREEDY: algorithm_name = "GULOSO"; break;
        case ALG_ASTAR:  algorithm_name = "A ESTRELA"; break;
        default:         algorithm_name = "DESCONHECIDO"; break;
    }
    snprintf(buf, sizeof(buf), "Algoritmo:");
    ssd1306_draw_string(ssd, 0, 40, buf);

    snprintf(buf, sizeof(buf), algorithm_name);
    ssd1306_draw_string(ssd, 0, 50, buf);

    // Renderiza no display
    render_on_display(ssd, &frame_area);
}


int manhattan(int x, int y, int gx, int gy) {
    return abs(x - gx) + abs(y - gy);
}

const char* get_direction_name(int dx, int dy) {
    if(dx == 0 && dy == -1) return "Baixo";
    if(dx == 0 && dy == 1)  return "Cima";
    if(dx == -1 && dy == 0) return "Dir";
    if(dx == 1 && dy == 0)  return "Esq";
    return "???";
}

int is_visited(int x, int y) {
    for(int i = 0; i < current_step; i++) {
        if(history[i].x == x && history[i].y == y) return 1;
    }
    return 0;
}

int greedy_step() {
    MoveInfo best, worse;
    if(!get_best_neighbor(current_x, current_y, &best, &worse))
        return 0;
    
    save_position(current_x, current_y);
    current_x = best.x;
    current_y = best.y;
    return (current_x == goal_x && current_y == goal_y);
}

void astar_init() {
    // Reinicia a fila do A* (e opcionalmente o histórico, se for um novo percurso)
    astar_front = -1;
    astar_rear = -1;
    int h = manhattan(current_x, current_y, goal_x, goal_y); 
    AStarNode start = {current_x, current_y, 0, h, h};
    astar_queue[++astar_rear] = start;
    save_position(current_x, current_y);
}

int astar_step() {
    if (astar_front == astar_rear)
        return 0; // Fila vazia
    
    int min_index = astar_front + 1;
    for (int i = min_index + 1; i <= astar_rear; i++) {
        if (astar_queue[i].f < astar_queue[min_index].f)
            min_index = i;
    }
    AStarNode temp = astar_queue[astar_front + 1];
    astar_queue[astar_front + 1] = astar_queue[min_index];
    astar_queue[min_index] = temp;

    AStarNode current = astar_queue[++astar_front];
    current_x = current.x;
    current_y = current.y;
    current_g_for_display = current.g;
    current_h_for_display = current.h;
    save_position(current_x, current_y);

    if (current.x == goal_x && current.y == goal_y)
        return 1;

    int dirs[4][2] = {{0,1}, {1,0}, {0,-1}, {-1,0}};
    for (int i = 0; i < 4; i++) {
        int nx = current.x + dirs[i][0];
        int ny = current.y + dirs[i][1];
        
        if (nx >= 0 && nx < MAZE_WIDTH && ny >= 0 && ny < MAZE_HEIGHT &&
            maze[ny][nx] != CELL_WALL && !is_visited(nx, ny)) {
            
            int g = current.g + 1;
            int h = manhattan(nx, ny, goal_x, goal_y);
            int f = g + h;
            AStarNode neighbor = {nx, ny, g, h, f};

            int insert_pos = astar_rear + 1;
            for (int j = astar_front + 1; j <= astar_rear; j++) {
                if (astar_queue[j].f > f) {
                    insert_pos = j;
                    break;
                }
            }
            for (int j = astar_rear; j >= insert_pos; j--) {
                astar_queue[j + 1] = astar_queue[j];
            }
            astar_queue[insert_pos] = neighbor;
            astar_rear++;
        }
    }
    return 0;
}

int get_best_neighbor(int x, int y, MoveInfo *best, MoveInfo *worse) {
    int directions[4][2] = {{0,-1}, {0,1}, {-1,0}, {1,0}};
    MoveInfo moves[4];
    int valid_moves = 0;
    int min_h = 9999;
    int max_h = -1;

    if (x == goal_x && y == goal_y)
        return 0;

    for (int i = 0; i < 4; i++) {
        int nx = x + directions[i][0];
        int ny = y + directions[i][1];
        
        if(nx >= 0 && nx < MAZE_WIDTH && ny >= 0 && ny < MAZE_HEIGHT) {
            if ((maze[ny][nx] == CELL_FREE || maze[ny][nx] == CELL_GOAL) && !is_visited(nx, ny)) {
                int h = manhattan(nx, ny, goal_x, goal_y);
                moves[valid_moves].x = nx;
                moves[valid_moves].y = ny;
                moves[valid_moves].h = h;
                if (h < min_h) min_h = h;
                if (h > max_h) max_h = h;
                valid_moves++;
            }
        }
    }

    if(valid_moves == 0)
        return 0;

    *best = moves[0];
    *worse = moves[0];
    for (int i = 0; i < valid_moves; i++) {
        if(moves[i].h < best->h)
            *best = moves[i];
        if(moves[i].h > worse->h)
            *worse = moves[i];
    }

    return 1;
}

void updateLEDMatrix(int current_h, int best, int worse) {
    printf("\n=== ESTADO DO LABIRINTO ===\n");
    
    for (int i = MAZE_HEIGHT - 1; i >= 0; i--) {
        for (int j = MAZE_WIDTH - 1; j >= 0; j--) {
            int idx;
            if (i % 2 == 0) {
                idx = i * MAZE_WIDTH + j;
            } else {
                idx = i * MAZE_WIDTH + (MAZE_WIDTH - 1 - j);
            }

            char cell;
            if (i == current_y && j == current_x) {
                npSetLED(idx, 0, 0, 255);
                cell = 'B';
            } else if (maze[i][j] == CELL_WALL) {
                npSetLED(idx, 255, 0, 0);
                cell = 'R';
            } else if (maze[i][j] == CELL_GOAL) {
                npSetLED(idx, 255, 255, 0);
                cell = 'Y';
            } else {
                npSetLED(idx, 0, 0, 0);
                cell = '.';
            }
            
            printf("%c ", cell);
        }
        printf("\n");
        
    }
    printf("Atual H: %d\n", current_h);
        printf("Melhor H: %d\n", best);
        printf("Pior H: %d\n", worse);
    printf("==========================\n\n");
    npWrite();
}

//--------------------------------------------------------------
// CONFIGURAÇÃO DE HARDWARE
void init_rgb_led() {
    gpio_init(LED_RED_PIN);
    gpio_init(LED_GREEN_PIN);
    gpio_init(LED_BLUE_PIN);
    gpio_set_dir(LED_RED_PIN, GPIO_OUT);
    gpio_set_dir(LED_GREEN_PIN, GPIO_OUT);
    gpio_set_dir(LED_BLUE_PIN, GPIO_OUT);
}

void init_buttons() {
    gpio_init(BUTTON_A_PIN);
    gpio_init(BUTTON_B_PIN);
    gpio_init(JOYSTICK_BUTTON_PIN);
    gpio_set_dir(BUTTON_A_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_A_PIN);
    gpio_pull_up(BUTTON_B_PIN);
    gpio_pull_up(JOYSTICK_BUTTON_PIN);
}

void init_buzzer() {
    gpio_init(BUZZER_PIN);
    gpio_set_dir(BUZZER_PIN, GPIO_OUT);
}

void init_joystick() {
    adc_init();
    adc_gpio_init(JOYSTICK_X_PIN);
    adc_gpio_init(JOYSTICK_Y_PIN);
}

void init_oled() {
    i2c_init(i2c, ssd1306_i2c_clock * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    ssd1306_init();
}

//--------------------------------------------------------------
// FUNÇÃO MAIN ATUALIZADA
int main() {
    stdio_init_all();
    init_rgb_led();
    init_buttons();
    init_buzzer();
    init_joystick();
    init_oled();
    npInit(WS2812_PIN);

    // Fase de introdução com matriz limpa
    clear_led_matrix();
    show_intro_screens();
    app_state = STATE_RUNNING;

    // Estado inicial
    save_position(current_x, current_y); 
    
    int current_h = manhattan(current_x, current_y, goal_x, goal_y);
    updateLEDMatrix(current_h, best_move.h, worse_move.h);
    get_best_neighbor(current_x, current_y, &best_move, &worse_move);
    updateOLEDDisplay(current_h, best_move.h, worse_move.h);

    while(true) {

        if (!gpio_get(JOYSTICK_BUTTON_PIN)) {
            // Troca o algoritmo
            gpio_put(LED_RED_PIN, 1);
            current_algorithm = (current_algorithm + 1) % ALG_TOTAL;
            update_algorithm_display();
    
            // Reinicia o labirinto
            current_x = 0;
            current_y = 0;
            current_step = 0;
            astar_front = -1;
            astar_rear = -1;
    
            updateOLEDDisplay(manhattan(current_x, current_y, goal_x, goal_y), 0, 0);
            updateLEDMatrix(manhattan(current_x, current_y, goal_x, goal_y), 0, 0);
            play_buzzer(1000, 300);
            // sleep_ms(300);
            gpio_put(LED_RED_PIN, 0);
        }

        current_h = manhattan(current_x, current_y, goal_x, goal_y);
        
        if(!gpio_get(BUTTON_A_PIN) && app_state == STATE_RUNNING) {
            sleep_ms(50); // Debounce
            if(current_h == 0) {
                // Troca de algoritmo quando na meta
                gpio_put(LED_GREEN_PIN, 1);
                current_algorithm = (current_algorithm + 1) % ALG_TOTAL;
                update_algorithm_display();
                
                current_x = 0;
                current_y = 0;
                current_step = 0;
                astar_front = -1;
                astar_rear = -1;
                
                updateOLEDDisplay(0, 0, 0);
                updateLEDMatrix(0, 0, 0);
                // sleep_ms(300);
                play_buzzer(1000, 300);
                gpio_put(LED_GREEN_PIN, 0);
                continue;
            }
            else {
                int result = 0;
                int best_val = 0, worse_val = 0;
                int current_val = 0;
                
                switch(current_algorithm) {
                    case ALG_GREEDY: {
                        result = greedy_step();
                        MoveInfo best, worse;
                        get_best_neighbor(current_x, current_y, &best, &worse);
                        best_val = best.h;
                        worse_val = worse.h;
                        current_val = manhattan(current_x, current_y, goal_x, goal_y);
                        break;
                    }
                    case ALG_ASTAR: {
                        // Se a fila estiver vazia (front == rear), reinicializa a fronteira
                        if(astar_front < 0 || astar_front == astar_rear) {
                            reinit_astar_frontier();
                        }
                        result = astar_step();
                        
                        int min_h = 9999, max_h = -1;
                        for(int i = astar_front + 1; i <= astar_rear; i++) {
                            int h = astar_queue[i].h;
                            if(h < min_h) min_h = h;
                            if(h > max_h) max_h = h;
                        }
                        
                        best_val = min_h;
                        worse_val = max_h;
                        current_val = manhattan(current_x, current_y, goal_x, goal_y);
                        break;
                    }
                }
                
                updateOLEDDisplay(current_val, best_val, worse_val);
                updateLEDMatrix(current_val, best_val, worse_val);
            }
            sleep_ms(200);
        }

        if(!gpio_get(BUTTON_B_PIN) && app_state == STATE_RUNNING) {
            sleep_ms(200);
            load_previous_position();
            
            MoveInfo prev_best, prev_worse;
            get_best_neighbor(current_x, current_y, &prev_best, &prev_worse);
            
            updateLEDMatrix(manhattan(current_x, current_y, goal_x, goal_y),
            prev_best.h, 
            prev_worse.h);
            updateOLEDDisplay(
                manhattan(current_x, current_y, goal_x, goal_y),
                prev_best.h, 
                prev_worse.h
            );
            
            while(!gpio_get(BUTTON_B_PIN)) sleep_ms(50);
        }

        sleep_ms(50);
    }

    return 0;
}
