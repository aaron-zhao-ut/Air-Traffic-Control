#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <iostream>
#include <cstdlib>
#include <math.h>
#include <ctime>
#include <string>
#include <deque>
#include <unordered_map>

const int SCREEN_WIDTH = 1200;
const int SCREEN_HEIGHT = 750;
int TCZ_RADIUS = 10000;
int PLANE_SIZE = 200;
double PLANE_SPEED = 140; // in m/s
int TRANSMIT_RATE = 10;     // in HZ (cycle/second)
int MAX_NUMBER_OF_PLANE = 10;
int MAX_NUMBER_OF_PLANE_IN_TOTAL = 1000;
int PLANE_COUNT = 0;
int NUMBER_OF_RUNWAY = 4;
int RUNWAY_W = 500, RUNWAY_L = 2500, RUNWAY_SAFETY_DISTANCE = 2500;
int PLANE_SAFETY_DISTANCE = 500; // in m
int HOLDING_RADIUS = 1000;

int SCREEN_CENTER_X, SCREEN_CENTER_Y;

SDL_Window *g_window = nullptr;
SDL_Surface *g_surface = nullptr;
SDL_Renderer *g_renderer = nullptr;

struct Traffic_Control_Zone
{
    int x, y;
    int r;
};

struct Runway
{
    SDL_Rect pos;
    int id;
    int bottom_center_x, bottom_center_y;
};
struct Plane
{
    int id;
    double x, y;
    double v_x, v_y;
    double target_angle;
    int taget_runway_id = -1;
    int state = 0; // 1 - normal flying , 2 - holding/circling , 3 - holding_entered, 4 - landing
};

std::deque<Plane> plane_deque;
std::unordered_map<int , std::deque<Runway> > runway_list; // 0 - free, 1 - occupied

// ----- helper functions -----
void draw_text(SDL_Renderer *surface, std::string input, int x, int y, Uint8 r = 255, Uint8 g = 255, Uint8 b = 255, Uint8 a = 0)
{
    std::string text_to_display = input;
    TTF_Font *font = TTF_OpenFont("/Users/Zhao/Desktop/Code_Test/ATC/fonts/Sans.ttf", 20);
    if (font == NULL)
    {
        std::cout << "Could not load font";
        return;
    }
    SDL_Color text_color = {r, g, b, a};
    SDL_Surface *tmpSurface = TTF_RenderText_Blended(font, text_to_display.c_str(), text_color);
    SDL_Texture *temTex = SDL_CreateTextureFromSurface(g_renderer, tmpSurface);
    int texW, texH;
    SDL_QueryTexture(temTex, NULL, NULL, &texW, &texH);
    SDL_Rect backgroundRect;
    backgroundRect.x = x;
    backgroundRect.y = y;
    backgroundRect.w = texW;
    backgroundRect.h = texH;
    SDL_RenderCopy(g_renderer, temTex, NULL, &backgroundRect);

    TTF_CloseFont(font);
    SDL_FreeSurface(tmpSurface);
    SDL_DestroyTexture(temTex);
}

void draw_circle(SDL_Renderer *surface, int n_cx, int n_cy, int radius)
{
    // if the first pixel in the screen is represented by (0,0) (which is in sdl)
    // remember that the beginning of the circle is not in the middle of the pixel
    // but to the left-top from it:

    double error = (double)-radius;
    double x = (double)radius - 0.5;
    double y = (double)0.5;
    double cx = n_cx - 0.5;
    double cy = n_cy - 0.5;

    while (x >= y)
    {
        SDL_RenderDrawPoint(surface, (int)(cx + x), (int)(cy + y));
        SDL_RenderDrawPoint(surface, (int)(cx + y), (int)(cy + x));

        if (x != 0)
        {
            SDL_RenderDrawPoint(surface, (int)(cx - x), (int)(cy + y));
            SDL_RenderDrawPoint(surface, (int)(cx + y), (int)(cy - x));
        }

        if (y != 0)
        {
            SDL_RenderDrawPoint(surface, (int)(cx + x), (int)(cy - y));
            SDL_RenderDrawPoint(surface, (int)(cx - y), (int)(cy + x));
        }

        if (x != 0 && y != 0)
        {
            SDL_RenderDrawPoint(surface, (int)(cx - x), (int)(cy - y));
            SDL_RenderDrawPoint(surface, (int)(cx - y), (int)(cy - x));
        }

        error += y;
        ++y;
        error += y;

        if (error >= 0)
        {
            --x;
            error -= x;
            error -= x;
        }
    }
}

void draw_triangle(SDL_Renderer *Renderer, Plane &p, int size = PLANE_SIZE)
{
    double angle = 0.0;
    int A_x, A_y, B_x, B_y, C_x, C_y;

    angle = p.target_angle;


    A_x = p.x + size * cos(angle * M_PI / 180);
    A_y = p.y - size * sin(angle * M_PI / 180);
    B_x = p.x + size * cos((angle + 135) * M_PI / 180);
    B_y = p.y - size * sin((angle + 135) * M_PI / 180);
    C_x = p.x + size * cos((angle - 135) * M_PI / 180);
    C_y = p.y - size * sin((angle - 135) * M_PI / 180);

    SDL_RenderDrawLine(Renderer, A_x, A_y, B_x, B_y);
    SDL_RenderDrawLine(Renderer, A_x, A_y, C_x, C_y);
    SDL_RenderDrawLine(Renderer, B_x, B_y, C_x, C_y);
}

bool inside_tcz(int x, int y)
{
    int c_x = SCREEN_CENTER_X;
    int c_y = SCREEN_CENTER_Y;

    return (sqrt(pow(x - c_x, 2) + pow(y - c_y, 2) * 1.0)) < TCZ_RADIUS - PLANE_SIZE;
}

double get_angle(double targer_x, double targer_y, double self_x, double self_y){
    double target_angle;
    if (targer_x - self_x == 0)
    {
        if (targer_y - self_y < 0){target_angle = 90.0;}
        else{target_angle = 270.0;}
    }
    else if (targer_y - self_y == 0)
    {
        if (targer_x - self_x < 0){target_angle = 180.0;}
        else{target_angle = 0.0;}
    }
    else
    {   
        target_angle = atan2(self_y - targer_y, targer_x - self_x) * 180 / M_PI;
    }
    
    return target_angle;
}
double get_plane_distance(Plane p1, Plane p2){
    return sqrt(pow(p1.x - p2.x , 2) + pow(p1.y - p2.y, 2));
}

void set_plane_v(Plane &p, double target_angle){
    p.target_angle = target_angle;
    p.v_x = PLANE_SPEED * cos(target_angle * M_PI / 180);
    p.v_y = -PLANE_SPEED * sin(target_angle * M_PI / 180);
}


// ----- operation -----
bool init()
{
    if (SDL_Init(SDL_INIT_VIDEO) < 0)
    {
        std::cout << "SDL could not start: " << SDL_GetError() << std::endl;
        return false;
    }

    g_window = SDL_CreateWindow("Air Traffic Control", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);

    if (g_window == NULL)
    {
        std::cout << "Could not create window: " << SDL_GetError();
        return false;
    }

    g_renderer = SDL_CreateRenderer(g_window, -1, SDL_RENDERER_ACCELERATED);

    if (g_renderer == NULL)
    {
        std::cout << "Could not create renderer: " << SDL_GetError();
        return false;
    }

    if (TTF_Init() < 0)
    {
        std::cout << "TTF_Init: " << TTF_GetError();
        return false;
    }

    return true;
}

Plane spawn_plane()
{
    Plane p;
    srand((unsigned)time(0));
    double random_angle = ((rand() % 360) + 1) * M_PI / 180;

    p.id = PLANE_COUNT+1;
    PLANE_COUNT ++;
    p.x = (double)SCREEN_CENTER_X + (cos(random_angle) * (TCZ_RADIUS));
    p.y = (double)SCREEN_CENTER_Y - (sin(random_angle) * (TCZ_RADIUS));

    for (int i = 0 ; i < plane_deque.size() ; i++){
        if (get_plane_distance(p , plane_deque[i]) <= PLANE_SAFETY_DISTANCE){
            random_angle += 30;
            p.x = (double)SCREEN_CENTER_X + (cos(random_angle) * (TCZ_RADIUS));
            p.y = (double)SCREEN_CENTER_Y - (sin(random_angle) * (TCZ_RADIUS));
        }
    }
    

    if (runway_list[0].size() > 0){
        p.state = 1;
        p.taget_runway_id = runway_list[0][0].id;
        double targer_x = runway_list[0][0].bottom_center_x;
        double targer_y = runway_list[0][0].bottom_center_y;
        runway_list[1].push_back(runway_list[0][0]);
        runway_list[0].pop_front();
        double target_angle = get_angle(targer_x, targer_y, p.x, p.y);

        p.target_angle = target_angle;
        set_plane_v(p , target_angle);
    }
    else {
        p.state = 3;
        double target_angle = get_angle (SCREEN_CENTER_X, SCREEN_CENTER_Y, p.x, p.y);
        set_plane_v(p , target_angle);
    }

    return p;
}

void spawn_runway(int runway_l, int runway_w, int runway_d)
{   
    for (int i = 0 ; i < NUMBER_OF_RUNWAY ; i++){
        Runway temp;
        temp.pos.h = runway_l;
        temp.pos.w = runway_w;
        temp.pos.x = (SCREEN_CENTER_X) - (runway_w/2) + ( (i - 0.5 * (NUMBER_OF_RUNWAY-1)) * (runway_d + runway_w)) ;
        temp.pos.y = SCREEN_CENTER_Y - runway_l / 2;
        temp.bottom_center_x = temp.pos.x + runway_w / 2;
        temp.bottom_center_y = temp.pos.y + runway_l;

        temp.id = i+1;
        runway_list[0].push_back(temp);
    }
    
}

// manual testing 
// void move(Plane &p, SDL_Scancode key_up, SDL_Scancode key_down, SDL_Scancode key_left, SDL_Scancode key_right)
// {
//     const Uint8 *keystates = SDL_GetKeyboardState(NULL);

//     if (keystates[key_up] && inside_tcz(p.x, p.y - PLANE_SPEED))
//     {
//         p.v_x = PLANE_SPEED * cos(45 * M_PI / 180);
//         p.v_y = -PLANE_SPEED * sin(45 * M_PI / 180);
//         p.x += p.v_x;
//         p.y += p.v_y;
//     }
//     else if (keystates[key_down] && inside_tcz(p.x, p.y + PLANE_SPEED))
//     {
//         p.v_x = PLANE_SPEED * cos(-135 * M_PI / 180);
//         p.v_y = -PLANE_SPEED * sin(-135 * M_PI / 180);
//         p.x += p.v_x;
//         p.y += p.v_y;
//     }
//     else if (keystates[key_left] && inside_tcz(p.x - PLANE_SPEED, p.y))
//     {
//         p.v_x = PLANE_SPEED * cos(135 * M_PI / 180);
//         p.v_y = -PLANE_SPEED * sin(135 * M_PI / 180);
//         p.x += p.v_x;
//         p.y += p.v_y;
//     }
//     else if (keystates[key_right] && inside_tcz(p.x + PLANE_SPEED, p.y))
//     {
//         p.v_x = PLANE_SPEED * cos(-45 * M_PI / 180);
//         p.v_y = -PLANE_SPEED * sin(-45 * M_PI / 180);
//         p.x += p.v_x;
//         p.y += p.v_y;
//     }
// }

void auto_movement(Plane &p)
{
    switch (p.state)
    {
    // normal flying 
    case 1:{
        p.x += p.v_x;
        p.y += p.v_y;

        double targer_x ;
        double targer_y ;
        Runway target;
        for (int i = 0 ; i < runway_list[1].size() ; i++){
            if (p.taget_runway_id == runway_list[1][i].id){
                target = runway_list[1][i];
                targer_x = target.bottom_center_x;
                targer_y = target.bottom_center_y;
                break;
            }
        }
        double target_angle = get_angle(targer_x, targer_y, p.x, p.y);

        for (int i = 0 ; i < plane_deque.size() ; i++){
            if (get_plane_distance(p , plane_deque[i]) <= PLANE_SAFETY_DISTANCE && p.id != plane_deque[i].id){
                double dot = p.x*plane_deque[i].x + p.y*plane_deque[i].y;     
                double det = p.x*plane_deque[i].y - p.y*plane_deque[i].x;     
                double angle = atan2(det, dot);
                if (angle < 45){
                    target_angle += 90;
                }
                else {
                    target_angle -= 90;
                }
            }
        }

        double offset = PLANE_SIZE+2;
        if (p.x >= target.pos.x - offset && p.x <= target.pos.x + RUNWAY_W + offset && p.y > target.bottom_center_y - offset && p.y < target.bottom_center_y + offset){
            p.state = 4;
            p.x = target.bottom_center_x;
            p.y = target.bottom_center_y;
            p.v_x = 0;
            p.v_y = - PLANE_SPEED;
            p.target_angle = 90;
        }
        else {
            set_plane_v(p , target_angle);
        }
        break;
    }

    // holding   
    case 2:{
        p.x += p.v_x;
        p.y += p.v_y;
        if (runway_list[0].size()>0){
            p.taget_runway_id = runway_list[0][0].id;
            runway_list[1].push_back(runway_list[0][0]);
            runway_list[0].pop_front();
            p.state = 1;
            break;
        }
        double target_angle = 90 - ((180 -(360 * PLANE_SPEED / (2 * M_PI * HOLDING_RADIUS)))/2);
        p.target_angle -= target_angle;
        p.v_x = PLANE_SPEED * cos(p.target_angle * M_PI / 180);
        p.v_y = -PLANE_SPEED * sin(p.target_angle * M_PI / 180);
        break;
    }

    // holding entered
    case 3:{
        p.x += p.v_x;
        p.y += p.v_y;
        double offset = 0.2 * HOLDING_RADIUS;
        if (sqrt(pow(SCREEN_CENTER_X - p.x , 2) + pow(SCREEN_CENTER_Y - p.y, 2)) < TCZ_RADIUS - (2 * HOLDING_RADIUS) - offset){
            p.state = 2;
            break;
        }
        double target_angle = get_angle(SCREEN_CENTER_X, SCREEN_CENTER_Y, p.x, p.y);
        set_plane_v(p , target_angle);
        break;
    }

    // landing
    case 4:{
        p.y += p.v_y;
        Runway target;
        int i;
        for (i = 0 ; i < runway_list[1].size() ; i++){
            if (p.taget_runway_id == runway_list[1][i].id){
                target = runway_list[1][i]; 
                break;
            }
        }
        if (p.y < target.pos.y + PLANE_SIZE) {
            p.state = -1;
            for (int pi = 0 ; pi < plane_deque.size() ; pi++){
                if (p.id == plane_deque[pi].id){
                    plane_deque.erase(plane_deque.begin()+pi);
                }
            }
            runway_list[0].push_back(target);
            runway_list[1].erase(runway_list[1].begin()+i);
        }
        break;
    }
        

    default:{break;}
        
    }
}

int main(int argc, char *argv[])
{
    // Basic UI
    // std::cout << "Hello! This is Aaron's mini ATC" << std::endl;
    // std::cout << "Please choose if you want to do any customization. (yes/no)" << std::endl; 
    // std::cout << "Answer no to watch default setting." << std::endl;
    // std::string input;
    // std::cin >> input;
    // if (input == "yes" || input == "y"){
    //     std::cout << "Please enter Traffic Control Zone Radius in KM" << std::endl;
    //     std::cin >> input;
    //     TCZ_RADIUS = 1000 * std::stoi(input);
    //     std::cout << "Please enter maximium possible number of plane at the same time." << std::endl;
    //     std::cin >> MAX_NUMBER_OF_PLANE;
    //     std::cout << "Please enter number of runways." << std::endl;
    //     std::cin >> NUMBER_OF_RUNWAY;
    //     std::cout << "Please enter runways width." << std::endl;
    //     std::cin >> RUNWAY_W;
    //     std::cout << "Please enter runways length." << std::endl;
    //     std::cin >> RUNWAY_L;
    // }
    // else if (input == "no" || input == "n"){

    // }
    // else {
    //     std::cout << "Please try again. ^_^ " << std::endl;
    // }

    int scale_factor = 27;

    // scale to fit in the screent
    TCZ_RADIUS /= scale_factor;
    PLANE_SIZE /= scale_factor;
    // PLANE_SPEED /= 10;
    PLANE_SPEED /= scale_factor;
    RUNWAY_W /= scale_factor;
    RUNWAY_L /= scale_factor;
    RUNWAY_SAFETY_DISTANCE /= scale_factor;
    PLANE_SAFETY_DISTANCE /= scale_factor;
    HOLDING_RADIUS /= scale_factor;

    double delay = (1000/TRANSMIT_RATE - 15.625)/15.625;

    if (!init())
    {
        printf("Failed to initialize!\n");
    }
    else
    {
        Traffic_Control_Zone tcz_1;
        SCREEN_CENTER_X = SCREEN_WIDTH / 2;
        SCREEN_CENTER_Y = SCREEN_HEIGHT / 2;

        tcz_1.x = SCREEN_CENTER_X;
        tcz_1.y = SCREEN_CENTER_Y;
        tcz_1.r = TCZ_RADIUS;

        spawn_runway(RUNWAY_L,RUNWAY_W,RUNWAY_SAFETY_DISTANCE);

        bool quit = false;
        SDL_Event e;

        while (!quit)
        {
            while (SDL_PollEvent(&e))
            {
                if (e.type == SDL_QUIT)
                {
                    quit = true;
                }
            }
            //Clear screen and set blackbackground
            SDL_SetRenderDrawColor(g_renderer, 0, 0, 0, 255);
            SDL_RenderClear(g_renderer);
            // SDL_SetRenderDrawColor(g_renderer, 255, 255, 255, 255);
            SDL_SetRenderDrawColor(g_renderer, 0, 230, 64, 1);

            // draw Traffic Control Zone
            draw_circle(g_renderer, tcz_1.x, tcz_1.y, tcz_1.r);

            // draw runways
            for (int i = 0 ; i < runway_list[0].size() ; i++ ){
                SDL_RenderDrawRect(g_renderer, &runway_list[0][i].pos);
            }
            for (int i = 0 ; i < runway_list[1].size() ; i++ ){
                SDL_RenderDrawRect(g_renderer, &runway_list[1][i].pos);
            }
            
            // move plane
            // move(plane_deque[0], SDL_SCANCODE_UP, SDL_SCANCODE_DOWN, SDL_SCANCODE_LEFT, SDL_SCANCODE_RIGHT);
            const Uint8 *keystates = SDL_GetKeyboardState(NULL);
            for (int i =  0; i < plane_deque.size(); i++){
                auto_movement(plane_deque[i]);
            }
            
            // spawn new plane if there are space
            if (plane_deque.size() < MAX_NUMBER_OF_PLANE)
            {
                plane_deque.push_back(spawn_plane());
            }
            
            // draw plane
            for (int i = 0; i < plane_deque.size(); i++)
            {
                draw_triangle(g_renderer, plane_deque[i]);
            }


            // debug dashboard
            // std::string tcz_info;
            // tcz_info = "circle:   x: " + std::to_string(tcz_1.x) + " y: " + std::to_string(tcz_1.y) + " r: " + std::to_string(tcz_1.r);
            // draw_text(g_renderer, tcz_info, 0, text_y+=20);
            // for (int i = 0 ; i < runway_list[0].size() ; i++ ){
            //     std::string r_info;
            //     r_info = "runway_" + std::to_string(i) + ": x: " + std::to_string(runway_list[0][i].pos.x) 
            //     + " y: " + std::to_string(runway_list[0][i].pos.y);
            //     draw_text(g_renderer, r_info, 0, text_y+=20);
            // }
            
            // for (int i = 0 ; i < plane_deque.size() ; i++ ){
            //     std::string p_info;
            //     p_info = "plane"+ std::to_string(i+1) + " target_dir: " + std::to_string(plane_deque[i].target_angle);
            //     draw_text(g_renderer, p_info, 0, text_y+=20);
            //     p_info = " target_runway: " + std::to_string(plane_deque[i].taget_runway_id)
            //     + " state: " + std::to_string(plane_deque[i].state);
            //     draw_text(g_renderer, p_info, 0, text_y+=20);
            // }
            
            
            SDL_RenderPresent(g_renderer);
            SDL_Delay(100);
        }
    }

    SDL_Quit();
    return 0;
}
