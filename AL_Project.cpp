#include<allegro5/allegro.h>
#include<allegro5/allegro_primitives.h>
#include<allegro5/allegro_image.h>
#include<vector>
#include<string>
#include<map>
#include<cmath>
#include<stack>
#include<iostream>
#include<fstream>
#include<random>
#include<jsoncpp/json/json.h>

using namespace std;

//GLOBALS==============================
const int WIDTH = 1280;
const int HEIGHT = 704;
int tile_width, tile_height;
enum KEYS {UP, DOWN, LEFT, RIGHT, SPACE};
bool keys[5] = {false, false, false, false, false};
vector<string> v_s;
int food_per_unit, food_rate;
int initial_num_prey, initial_num_predator;
int adult_age_prey, adult_age_predator;
int dist_to_reproduce_predator, dist_to_reproduce_prey;
int steps_hungry_predator, steps_hungry_prey;
int affinity_threshold;
float boid1_param, boid2_param, boid3_param;
float tend_food_param, running_food_param, run_from_pred_param;
float tend_prey_param, tend_mate_param;
bool run_when_hungry;
Json::Value prey_conf_root, env_conf_root, predator_conf_root;
default_random_engine generator(time(0));

//defining the structures needed for the game
typedef struct point point;
typedef struct tree tree;
typedef struct predator predator;
typedef struct prey prey;
typedef struct tile tile;
typedef struct vect vect;

struct vect
{
    float v_x, v_y;
    vect(float n_x, float n_y) : v_x(n_x),v_y(n_y) {}
    inline vect operator=(vect b)
    {
        v_x = b.v_x;
        v_y = b.v_y;
        return b;
    }
    inline vect operator+(vect b)
    {
        return {b.v_x + v_x, b.v_y + v_y};
    }
    inline vect operator-(vect b)
    {
        return {v_x - b.v_x, v_y - b.v_y};
    }
    inline vect operator*(float b)
    {
        return {v_x * b, v_y * b};
    }
    inline vect operator/(float b)
    {
        return {v_x / b, v_y / b};
    }
};
struct point
{
    float x, y;
};
struct tree
{
    point* p;
    char axiom;
    int depth;
    int length;
    int id;
    float angle;
    map<char, string>* rules;
};
struct predator
{
    tile* tile_pos;
    vect velocity;
    vect pos;
    int metabolism, max_age, capacity, cur_age, cur_food;
    bool is_dead;
    int skin;
    int trans;
    float base_speed, run_speed;
    int vision_rad;
    bool is_male;
    bool is_attacking;
    int attack_cur_steps, attack_max_steps;
    int rest_steps;
    prey* predating;
};
struct tile
{
    vect* pos;
    int food;
    bool has_prey;
    bool has_predator;
    prey* prey_on_tile;
    predator* pred_on_tile;
};
struct prey
{
    tile* tile_pos;
    vect velocity;
    vect pos;
    int metabolism, max_age, capacity, cur_age, cur_food;
    bool is_dead;
    //discretized represantions of skin [1-5]
    int skin;
    //discretized represantion of affine transformation [1-5]
    int trans;
    float base_speed, run_speed;
    int vision_rad;
    bool is_male;
    bool is_being_attacked;
    bool has_been_eaten;
};
struct season
{
    int x,y;
    int duration;
    string name;
};

//defining the signatures of the methods to be used
point* point_init(float n_x, float n_y);
tree *tree_init(point* n_p, char n_axiom,
                int n_depth, int n_length,
                std::map<char,std::string>* n_rules,
                int n_id, float n_angle);
prey* prey_init(tile* tile, int skin = -1, int trans = -1);
predator* predator_init(tile* tile, int skin = -1, int trans = -1);
void prey_draw(prey* prey);
void prey_destroy(prey* prey);
void prey_move(vector<prey>& v_prey, vector< vector<tile> >& tile_mat,
               int max_x, int max_y);
void prey_reproduce(vector<prey>& v_prey, vector< vector<tile> >&tile_mat,
                    int max_x, int max_y);
bool prey_child_born(vector<prey>& v_prey, vector< vector<tile> >& tile_mat,
                     vect pos, int skin, int trans, int max_x, int max_y);
int prey_get_distance(prey* cur_prey, prey* other_prey);
void predator_draw(predator* predator);
void predator_destroy(predator* predator);
void predator_move(vector<predator>& v_predator, vector< vector<tile> >& tile_mat,
                   int max_x, int max_y);
void predator_reproduce(vector<predator>& v_predator, vector< vector<tile> >&tile_mat,
                        int max_x, int max_y);
bool predator_child_born(vector<predator>& v_predator, vector< vector<tile> >& tile_mat,
                         vect pos, int skin, int trans, int max_x, int max_y);
void predator_eat_prey(predator* predator, prey* prey, vector< vector<tile> >& tile_mat);
int predator_get_distance(predator* cur_predator, predator* other_predator);
void tree_get_full_rule(tree* tree);
void tree_draw(tree* tree);
void tree_destroy(tree* tree);
point* next_point(point* point, float angle, float length);
void process_string(tree* tree, vector<char>* old_str, vector<char>* to_return);
void apply_rules(tree* tree, char c, vector<char>* v);
void add_sand(vector< vector<tile> >& tile_mat, int i, int j,
              int n, int m, int max_food, int to_add);
int get_tile_distance(tile* tile1, tile* tile2, int max_x, int max_y);
pair<float,float> get_tile_center(tile* tile);
tile* get_tile_from_coord(int x, int y, vector< vector<tile> >& tile_mat);
tile* tile_swap_prey(vector< vector<tile> >& tile_mat, prey* cur_tile, tile* next_tile,
                     int max_x, int max_y);
tile* tile_swap_predator(vector< vector<tile> >& tile_mat, predator* cur_tile, tile* next_tile,
                         int max_x, int max_y);
///returns a random value distributed as a power law with param 'param' between x0 and x1
int power_law_val(int param, int x0, int x1);
vect boids_rule1(vector<prey>& v_prey, int cur_prey_ind);
vect boids_rule2(vector<prey>& v_prey, int cur_prey_ind);
vect boids_rule3(vector<prey>& v_prey, int cur_prey_ind);
vect boids_tend_to_food(prey* prey, vect place);
vect boids_run_from_pred(prey* prey, vect pred);
vect tend_to_prey(predator* predator, vect prey_tile);
vect tend_to_mate(predator* predator, vect mate_tile);
vect get_nearest_food_prey(prey * prey, vector< vector<tile> >& tile_mat, int max_x, int max_y);
vect get_nearest_predator(prey* prey, vector< vector<tile> >& tile_mat, int max_x, int max_y);
vect get_nearest_food_predator(predator* predator, vector< vector<tile> >& tile_mat, int max_x, int max_y);
vect get_nearest_mate(predator* prey, vector< vector<tile> >& tile_mat, int max_x, int max_n);
float vect_norm(vect v);

int main(void)
{
    //reading JSON configuration files
    ifstream env_ifs("Environment.json");
    ifstream env_prey("Prey.json");
    ifstream env_predator("Predator.json");
    Json::Reader reader_env;
    Json::Reader reader_prey;
    Json::Reader reader_predator;
    reader_env.parse(env_ifs, env_conf_root);
    reader_prey.parse(env_prey, prey_conf_root);
    reader_predator.parse(env_predator, predator_conf_root);

    //primitive variables
    bool done = false;
    bool redraw = true;
    const int FPS = 5;

    //initializing a tree_vector
    vector<tree*> tree_vector;
    map<char,string> rules;

    //building environment trees from configuration
    Json::Value trees_conf = env_conf_root["trees"];
    float x,y,angle;
    int depth,length;
    char axiom;
    for(int i = 0; i < trees_conf.size(); i++)
    {
        x = trees_conf[i]["x"].asFloat();
        y = trees_conf[i]["y"].asFloat();
        angle = trees_conf[i]["angle"].asFloat();
        depth = trees_conf[i]["depth"].asInt();
        length = trees_conf[i]["length"].asInt();
        axiom = trees_conf[i]["axiom"].asString()[0];
        //popoulating the rules map from the configuration
        rules.clear();
        Json::Value rules_conf = trees_conf[i]["rules"];
        for(int j = 0; j < rules_conf.size(); j++)
        {
            rules[rules_conf[j]["antecedente"].asString()[0]] =
                rules_conf[j]["consecuente"].asString();
        }
        tree_vector.push_back(
            tree_init(point_init(x, y), axiom,
                      depth, length, &rules, i, angle));
    }

    //creating a map of tiles
    Json::Value tile_conf = env_conf_root["tiles"];
    tile_width = tile_conf["width"].asInt();
    tile_height = tile_conf["heigth"].asInt();
    int max_food = tile_conf["max_food"].asInt();
    vector< vector<tile> > tiles_mat(WIDTH / tile_width,
                                     vector<tile>(HEIGHT / tile_height));
    //initializing all the tiles to have 0 food
    for(int i = 0; i < (WIDTH / tile_width); i++)
    {
        for(int j = 0; j < (HEIGHT / tile_height); j++)
        {
            tiles_mat[i][j].food = 0;
            tiles_mat[i][j].pos = new vect(i,j);
            tiles_mat[i][j].has_prey = false;
            tiles_mat[i][j].has_predator = false;
        }
    }

    //reading the configuration for the food rate
    food_rate = env_conf_root["food_per_second"].asInt();
    food_per_unit = env_conf_root["food_per_unit"].asInt();
    run_when_hungry = env_conf_root["run_when_hungry"].asBool();

    //reading information about seasons
    Json::Value seasons_conf = env_conf_root["seasons"];
    season seasons_v[seasons_conf.size()];
    for(int i = 0; i < seasons_conf.size(); i++)
    {
        seasons_v[i].x = seasons_conf[i]["x"].asInt();
        seasons_v[i].y = seasons_conf[i]["y"].asInt();
        seasons_v[i].duration = seasons_conf[i]["duration"].asInt();
        seasons_v[i].name = seasons_conf[i]["name"].asString();
    }
    int season_c = 0;
    int fps_c = 0;
    int seconds_c = 0;
    int current_season = 0;

    //reading information about preys
    initial_num_prey = prey_conf_root["initial_number"].asInt();
    adult_age_prey = prey_conf_root["adult_age"].asInt();
    dist_to_reproduce_prey = prey_conf_root["dist_to_reproduce"].asInt();
    boid1_param = prey_conf_root["boids1_param"].asFloat();
    boid2_param = prey_conf_root["boids2_param"].asFloat();
    boid3_param = prey_conf_root["boids3_param"].asFloat();
    tend_food_param = prey_conf_root["tend_to_food_param"].asFloat();
    run_from_pred_param = prey_conf_root["run_from_pred_param"].asFloat();
    running_food_param = prey_conf_root["running_food_param"].asFloat();
    steps_hungry_prey = prey_conf_root["steps_to_hungry"].asInt();

    uniform_int_distribution<int> x_gen(0, (WIDTH / tile_width) - 1);
    uniform_int_distribution<int> y_gen(0, (HEIGHT / tile_height) - 1);
    //creating preys from the given configuration
    vector<prey> prey_vector;
    tile* to_put;

    for(int i = 0; i < initial_num_prey; i++)
    {
        //taking a random tile to put the prey
        do
        {
            to_put = &tiles_mat[x_gen(generator)][y_gen(generator)];
        }
        while(to_put->has_prey || to_put->has_predator);
        //cout<<"to_put pos: ("<<to_put->pos->v_x<<" , "<<to_put->pos->v_y<<")"<<endl;

        prey* to_add = prey_init(to_put);
        //cout<<"to_add address: "<<to_add<<endl;
        prey_vector.push_back(*to_add);
    }
    //cout<<"finishing adding preys"<<endl;

    //reading information about preys
    initial_num_predator = predator_conf_root["initial_number"].asInt();
    adult_age_predator = predator_conf_root["adult_age"].asInt();
    dist_to_reproduce_predator = predator_conf_root["dist_to_reproduce"].asInt();
    steps_hungry_predator = predator_conf_root["steps_to_hungry"].asInt();
    tend_prey_param = predator_conf_root["tend_prey_param"].asInt();
    tend_mate_param = predator_conf_root["tend_mate_param"].asInt();
    affinity_threshold = predator_conf_root["affinity_threshold"].asInt();
    //creating preys from the given configuration
    vector<predator> predator_vector;

    for(int i = 0; i < initial_num_predator; i++)
    {
        //taking a random tile to put the prey
        do
        {
            to_put = &tiles_mat[x_gen(generator)][y_gen(generator)];
        }
        while(to_put->has_prey || to_put->has_predator);
        //cout<<"to_put pos: ("<<to_put->pos->v_x<<" , "<<to_put->pos->v_y<<")"<<endl;
        predator* to_add = predator_init(to_put);
        //cout<<"to_add address: "<<to_add<<endl;
        predator_vector.push_back(*to_add);
    }
    //cout<<"finishing adding predators"<<endl;

    //Allegro variables
    ALLEGRO_DISPLAY *display = NULL;
    ALLEGRO_EVENT_QUEUE *event_queue = NULL;
    ALLEGRO_TIMER *timer = NULL;
    ALLEGRO_TIMER *food_timer = NULL;

    //Initialization Functions
    if(!al_init())										//initialize Allegro
        return -1;
    display = al_create_display(WIDTH, HEIGHT);			//create our display object
    if(!display)										//test display object
        return -1;

    al_init_primitives_addon();
    al_install_keyboard();
    al_init_image_addon();
    event_queue = al_create_event_queue();
    timer = al_create_timer(1.0 / FPS);
    food_timer = al_create_timer(1.0 / food_rate);

    srand(time(NULL));
    al_register_event_source(event_queue, al_get_keyboard_event_source());
    al_register_event_source(event_queue, al_get_timer_event_source(timer));
    al_register_event_source(event_queue, al_get_timer_event_source(food_timer));
    al_register_event_source(event_queue, al_get_display_event_source(display));
    al_start_timer(timer);
    al_start_timer(food_timer);

    while(!done)
    {
        ALLEGRO_EVENT ev;
        al_wait_for_event(event_queue, &ev);
        if(ev.type == ALLEGRO_EVENT_TIMER)
        {
            if(ev.timer.source == timer)
            {
                redraw = true;
                /*if(keys[UP])
                    MoveShipUp(ship);*/
                fps_c++;
                if (fps_c == FPS)
                {
                    fps_c = 0;
                    seconds_c++;
                    season_c++;
                    //adding one year to the preys
                    for(int i = 0; i < prey_vector.size(); i++)
                    {
                        prey_vector[i].cur_age++;
                    }
                    //preys will reproduce each year
                    prey_reproduce(prey_vector, tiles_mat, WIDTH / tile_width, HEIGHT / tile_height);
                    predator_reproduce(predator_vector, tiles_mat, WIDTH / tile_width, HEIGHT / tile_height);
                }
            }
            //it is time to add food!!
            else if(ev.timer.source == food_timer)
            {
                //cout<<"time to food"<<endl;
                season cur_season = seasons_v[current_season];
                if (season_c > cur_season.duration)
                {
                    season_c = 0;
                    current_season++;
                    if(current_season == seasons_conf.size())
                        current_season = 0;
                }
                int x = cur_season.x / tile_width;
                int y = cur_season.y / tile_height;

                //cout<<"x: "<<x<<" y: "<<y<<"cur food: "<<tiles_mat[x][y].food<<endl;
                add_sand(tiles_mat, x, y, WIDTH / tile_width,
                         HEIGHT / tile_height, max_food, 1);
            }
        }
        else if(ev.type == ALLEGRO_EVENT_DISPLAY_CLOSE)
        {
            done = true;
        }
        else if(ev.type == ALLEGRO_EVENT_KEY_DOWN)
        {
            switch(ev.keyboard.keycode)
            {
            case ALLEGRO_KEY_ESCAPE:
                done = true;
                break;
                /*case ALLEGRO_KEY_UP:
                	keys[UP] = true;
                	break;
                */
            }
        }
        else if(ev.type == ALLEGRO_EVENT_KEY_UP)
        {
            switch(ev.keyboard.keycode)
            {
            case ALLEGRO_KEY_ESCAPE:
                done = true;
                break;
                /*case ALLEGRO_KEY_UP:
                	keys[UP] = false;
                	break;*/
            }
        }

        if(redraw && al_is_event_queue_empty(event_queue))
        {
            redraw = false;

            //1) drawing tiles
            for(int i = 0; i < (WIDTH / tile_width); i++)
            {
                for(int j = 0; j < (HEIGHT / tile_height); j++)
                {
                    float ini_x = i * tile_width;
                    float ini_y = j * tile_height;
                    //does this tile have food?
                    if (tiles_mat[i][j].food > 0)
                    {
                        //fill with a scale of blues
                        al_draw_filled_rectangle(ini_x, ini_y, ini_x + tile_width,
                                                 ini_y + tile_height, al_map_rgb(0, 0,
                                                         tiles_mat[i][j].food * (256 / max_food)));
                    }
                    else
                    {
                        //fill with savannah color
                        al_draw_filled_rectangle(ini_x, ini_y, ini_x + tile_width,
                                                 ini_y + tile_height, al_map_rgb(248, 199, 23));
                    }
                }
            }
            //2) drawing trees
            for(int i = 0; i < tree_vector.size(); i++)
            {
                tree_draw(tree_vector[i]);
            }

            //3) drawing preys
            //deleting dead preys
            for(int i = 0; i < prey_vector.size(); )
            {
                if(prey_vector[i].is_dead)
                {
                    prey_vector.erase(prey_vector.begin() + i);
                    tiles_mat[prey_vector[i].pos.v_x][prey_vector[i].pos.v_y].has_prey = false;
                }
                else
                    i++;
            }
            for(int i = 0; i < prey_vector.size(); i++)
            {
                if(prey_vector[i].cur_food < 0)
                {
                    cout<<"zebra: "<<i<<" died of starving"<<endl;
                    prey_vector[i].is_dead = true;
                    continue;
                }
                else if(prey_vector[i].cur_age >= prey_vector[i].max_age)
                {
                    cout<<"zebra i: "<<i<<" died of old"<<endl;
                    prey_vector[i].is_dead = true;
                    continue;
                }
                else if(prey_vector[i].has_been_eaten)
                {
                    cout<<"zebra i: "<<i<<" has been eaten"<<endl;
                    prey_vector[i].is_dead = true;
                    continue;
                }

                prey_draw(&prey_vector[i]);
                prey_vector[i].cur_food -= prey_vector[i].metabolism;
            }


            //4) drawing predators
            for(int i = 0; i < predator_vector.size(); )
            {
                if(predator_vector[i].is_dead)
                {
                    predator_vector.erase(predator_vector.begin() + i);
                    tiles_mat[predator_vector[i].pos.v_x][predator_vector[i].pos.v_y].has_predator = false;
                }
                else
                    i++;
            }
            for(int i = 0; i < predator_vector.size(); i++)
            {
                //is the prey dead?
                if(predator_vector[i].cur_food < 0 ||
                        predator_vector[i].cur_age >= predator_vector[i].max_age)
                {
                    if(predator_vector[i].cur_food < 0)
                        cout<<"cheetah i: "<<i<<" died of starving"<<endl;
                    else
                        cout<<"cheetah i: "<<i<<" died of old"<<endl;
                    predator_vector[i].is_dead = true;
                    continue;
                }
                predator_draw(&predator_vector[i]);
                predator_vector[i].cur_food -= predator_vector[i].metabolism;
            }
            prey_move(prey_vector, tiles_mat, WIDTH / tile_width, HEIGHT / tile_height);
            predator_move(predator_vector, tiles_mat,  WIDTH / tile_width, HEIGHT / tile_height);
            al_flip_display();
            al_clear_to_color(al_map_rgb(0,0,0));

        }

    }


    al_destroy_event_queue(event_queue);
    al_destroy_timer(timer);
    al_destroy_display(display);						//destroy our display object

    return 0;
}

point* point_init(float n_x, float n_y)
{
    point* new_point = (point*)malloc(sizeof(point));
    new_point->x = n_x;
    new_point->y = n_y;
}
tree* tree_init(point* n_p, char n_axiom,
                int n_depth, int n_length,
                map<char,string>* n_rules,
                int n_id, float n_angle)
{
    tree* new_tree = (tree*)malloc(sizeof(tree));
    new_tree->p = n_p;
    new_tree->depth = n_depth;
    new_tree->length = n_length;
    new_tree->axiom = n_axiom;
    new_tree->id = n_id;
    new_tree->rules = n_rules;
    new_tree->angle = n_angle;
    tree_get_full_rule(new_tree);
    return new_tree;
}
prey* prey_init(tile* tile, int skin, int trans)
{
    Json::Value spec_conf = prey_conf_root["species"];
    int pow_law_param = prey_conf_root["power_law_param"].asInt();
    bool total_vision = prey_conf_root["total_vision"].asBool();
    prey* new_prey = (prey*)malloc(sizeof(prey));
    if(skin == -1)
        new_prey->skin = 6 - power_law_val(pow_law_param, 1, 6);
    else
        new_prey->skin = skin;
    if(trans == -1)
        new_prey->trans = 6 - power_law_val(pow_law_param, 1, 6);
    else
        new_prey->trans = trans;
    //cout<<"new prey: skin: "<<new_prey->skin<<" trans: "<<new_prey->trans<<endl;
    int spec_ind = (new_prey->skin - 1) * 5 + (new_prey->trans);
    new_prey->metabolism = spec_conf[spec_ind]["metabolism"].asInt();
    new_prey->capacity = spec_conf[spec_ind]["capacity"].asInt();
    new_prey->max_age = spec_conf[spec_ind]["avg_age"].asInt();
    normal_distribution<double> distribution((float)new_prey->max_age, 10.0);
    new_prey->max_age = (int)distribution(generator);
    new_prey->cur_age = 0;
    new_prey->cur_food = new_prey->capacity;
    new_prey->is_dead = false;
    new_prey->has_been_eaten = false;
    new_prey->is_being_attacked = false;
    new_prey->tile_pos = tile;
    vect v_pos(tile->pos->v_x,tile->pos->v_y);
    new_prey->pos = v_pos;
    new_prey->velocity = *new vect(1,1);
    //cout<<"after init: "<<new_prey->tile_pos->x<<" , "<<new_prey->tile_pos->y<<endl;
    new_prey->run_speed = spec_conf[spec_ind]["max_speed"].asInt();
    new_prey->base_speed = spec_conf[spec_ind]["base_speed"].asInt();
    new_prey->vision_rad = spec_conf[spec_ind]["vision"].asInt();
    if(total_vision)
        new_prey->vision_rad *= 100;
    //is this a male?
    double r = ((double) rand() / (RAND_MAX));
    if(r > 0.5)
        new_prey->is_male = true;
    //adding the prey to the tile
    tile->has_prey = true;
    tile->prey_on_tile = new_prey;

    return new_prey;
}
predator* predator_init(tile* tile, int skin, int trans)
{
    Json::Value spec_conf = predator_conf_root["species"];
    int pow_law_param = predator_conf_root["power_law_param"].asInt();
    bool total_vision = predator_conf_root["total_vision"].asBool();
    predator* new_predator = (predator*)malloc(sizeof(predator));
    if(skin == -1)
        new_predator->skin = 6 - power_law_val(pow_law_param, 1, 6);
    else
        new_predator->skin = skin;
    if(trans == -1)
        new_predator->trans = 6 - power_law_val(pow_law_param, 1, 6);
    else
        new_predator->trans = trans;
    //cout<<"new prey: skin: "<<new_predator->skin<<" trans: "<<new_predator->trans<<endl;
    int spec_ind = (new_predator->skin - 1) * 5 + (new_predator->trans);
    new_predator->metabolism = spec_conf[spec_ind]["metabolism"].asInt();
    new_predator->capacity = spec_conf[spec_ind]["capacity"].asInt();
    new_predator->max_age = spec_conf[spec_ind]["avg_age"].asInt();
    new_predator->attack_max_steps = spec_conf[spec_ind]["steps_bef_tired"].asInt();
    new_predator->rest_steps = spec_conf[spec_ind]["steps_to_rest"].asInt();
    normal_distribution<double> distribution((float)new_predator->max_age, 10.0);
    new_predator->max_age = (int)distribution(generator);
    new_predator->cur_age = 0;
    new_predator->cur_food = new_predator->capacity;
    new_predator->is_dead = false;
    new_predator->tile_pos = tile;
    new_predator->is_attacking = false;
    new_predator->attack_cur_steps = 0;
    vect v_pos(tile->pos->v_x,tile->pos->v_y);
    new_predator->pos = v_pos;
    new_predator->velocity = *new vect(1,1);
    //cout<<"after init: "<<new_predator->tile_pos->x<<" , "<<new_predator->tile_pos->y<<endl;
    new_predator->run_speed = spec_conf[spec_ind]["max_speed"].asInt();
    new_predator->base_speed = spec_conf[spec_ind]["base_speed"].asInt();
    new_predator->vision_rad = spec_conf[spec_ind]["vision"].asInt();
    if(total_vision)
        new_predator->vision_rad *= 100;
    //is this a male?
    double r = ((double) rand() / (RAND_MAX));
    if(r > 0.5)
        new_predator->is_male = true;
    //adding the prey to the tile
    tile->has_predator = true;
    tile->pred_on_tile = new_predator;

    return new_predator;
}
void tree_get_full_rule(tree* tree)
{
    vector<char> start_string(1,tree->axiom);
    vector<char> end_string;
    for (int i = 0; i < tree->depth; i++)
    {
        process_string(tree, &start_string, &end_string);
        start_string = end_string;
    }
    string to_push = "";
    for(int i = 0; i < end_string.size(); i++)
        to_push.push_back(end_string[i]);
    v_s.push_back(to_push);
}
void process_string(tree* tree, vector<char>* old_str,
                    vector<char>* end_str)
{
    vector<char> new_str;
    for(int i = 0; i < old_str->size(); i++)
    {
        apply_rules(tree, (*old_str)[i], &new_str);
    }
    (*end_str) = new_str;
}
void apply_rules(tree* tree, char c, vector<char>* v)
{
    if (tree->rules->count(c) > 0)
    {
        string s = (*tree->rules)[c];
        for(int i = 0; i < s.length(); i++)
        {
            (*v).push_back(s[i]);
        }
    }
    else
    {
        (*v).push_back(c);
    }
}
void tree_draw(tree* tree)
{
    string s_ini = v_s[tree->id];
    point* cur_point = tree->p;
    float cur_angle = 90.0;
    stack< pair<point*, float> > s;
    for (int i = 0; i < s_ini.length(); i++)
    {
        char c = s_ini[i];
        switch(c)
        {
        case 'F':
        {
            //parsing the angle in the case its negative
            float angle_d = rand() % 10 - 5;
            angle_d /= 50.0;
            cur_angle += angle_d;
            if(cur_angle < 0)
            {
                cur_angle = 360 + cur_angle;
            }
            if(cur_angle > 360)
            {
                cur_angle = cur_angle - 360;
            }

            point* n_point = next_point(cur_point, cur_angle, tree->length);
            al_draw_line(cur_point->x, cur_point->y, n_point->x, n_point->y,
                         al_map_rgb(0,255,0), 2);

            cur_point = n_point;
            break;
        }
        case '+':
        {
            cur_angle += tree->angle;
            break;
        }
        case '-':
        {
            cur_angle -= tree->angle;
            break;
        }
        case '[':
        {
            s.push(make_pair(cur_point, cur_angle));
            break;
        }
        case ']':
        {
            cur_point = s.top().first;
            cur_angle = s.top().second;
            s.pop();
            break;
        }
        }
    }
}
void tree_destroy(tree* tree)
{
    if(tree)
        free(tree);
}
void prey_draw(prey* prey)
{
    pair<float,float> tile_center = get_tile_center(prey->tile_pos);
    int spec_ind = (prey->skin - 1) * 5 +  prey->trans;
    al_draw_filled_circle(tile_center.first, tile_center.second,
                          tile_width / 2.0, al_map_rgb(spec_ind * (256 / 25),0,0));
}
void prey_destroy(prey* prey)
{
    if(prey)
        free(prey);
}
int prey_get_distance(prey* cur_prey, prey* other_prey)
{
    return abs(cur_prey->skin - other_prey->skin) +
           abs(cur_prey->trans - other_prey->trans);
}
void prey_reproduce(vector<prey>& v_prey, vector< vector<tile> >&tile_mat,
                    int max_x, int max_y)
{
    for(int i = 0; i < v_prey.size(); i++)
    {
        prey* cur_prey = &v_prey[i];
        int min_dist = -1, min_ind = -1;
        if(cur_prey->cur_age > adult_age_prey &&
            cur_prey->cur_food >= cur_prey->capacity / 2)
        {
            for(int j = 0; j < v_prey.size(); j++)
            {
                if(j != 1)
                {
                    prey* other_prey = &v_prey[j];
                    if(cur_prey->is_male^other_prey->is_male &&
                            vect_norm(cur_prey->pos - other_prey->pos) < dist_to_reproduce_prey)
                    {
                        int spec_dist = prey_get_distance(cur_prey, other_prey);
                        if(min_dist == -1)
                        {
                            min_dist = spec_dist;
                            min_ind = j;
                            continue;
                        }
                        if(spec_dist < min_dist)
                        {
                            min_dist = spec_dist;
                            min_ind = j;
                        }
                    }
                }
            }
            //found some lover :p
            if(min_dist != -1)
            {
                prey* lover = &v_prey[min_ind];
                float prob = min_dist / 20;

                double r = rand() % RAND_MAX;
                if(r > prob)
                {
                    int tries = 0;
                    vect middle = (cur_prey->pos + lover->pos) / 2.0;
                    int skin = (cur_prey->skin + lover->skin) / 2;
                    int trans = (cur_prey->trans + lover->trans) / 2;
                    bool success =
                        prey_child_born(v_prey, tile_mat, middle, skin, trans,
                                        max_x, max_y);
                    if(success)
                    {
                        cout<<"child zebra was born at: ("<<middle.v_x<<" , "<<middle.v_y<<")"<<endl;
                    }
                }
            }
        }
    }
}
bool prey_child_born(vector<prey>& v_prey, vector< vector<tile> >& tile_mat,
                     vect pos, int skin, int trans, int max_x, int max_y)
{
    bool success = true;
    int tries = 0;
    tile* next_tile = &tile_mat[pos.v_x][pos.v_y];
    while((next_tile->has_prey || next_tile->has_predator))
    {
        if(tries == 1)
        {
            success =false;
            break;
        }
        double n_rand = ((double) rand() / (RAND_MAX));
        int to_add = rand() % 3;
        if(n_rand > 0.5)
        {
            if(next_tile->pos->v_x + to_add < max_x)
            {
                next_tile = &tile_mat[next_tile->pos->v_x + to_add][next_tile->pos->v_y];

            }
            else
            {
                next_tile = &tile_mat[next_tile->pos->v_x - to_add][next_tile->pos->v_y];
            }
        }
        else
        {
            if(next_tile->pos->v_y + to_add < max_y)
            {

                next_tile = &tile_mat[next_tile->pos->v_x][next_tile->pos->v_y + to_add];
            }
            else
            {
                next_tile = &tile_mat[next_tile->pos->v_x][next_tile->pos->v_y - to_add];
            }
        }
        tries++;
    }
    if(success)
    {
        v_prey.push_back(*prey_init(next_tile, skin, trans));
    }
    return success;
}
void prey_move(vector<prey>& v_prey, vector< vector<tile> >& tile_mat,
               int max_x, int max_y)
{
    vect v1(0.0,0.0),v2(0.0,0.0),v3(0.0,0.0),v4(0.0,0.0), v5(0.0,0.0);
    for(int i = 0 ; i < v_prey.size(); i++)
    {
        prey* cur_prey = &v_prey[i];
        //should I take food?
        if(cur_prey->tile_pos->food > 0 && cur_prey->cur_food < cur_prey->capacity)
        {
            cur_prey->tile_pos->food--;
            cur_prey->cur_food+= food_per_unit;
            continue;
        }
        //cout<<"prey i: "<<i<<" before pos: ("<<v_prey[i].pos.v_x<<" , "<<v_prey[i].pos.v_y<<")"<<endl;
        v1 = boids_rule1(v_prey, i);
        v2 = boids_rule2(v_prey, i);
        v3 = boids_rule3(v_prey, i);
        //tend to the food
        vect nearest_food_prey = get_nearest_food_prey(cur_prey, tile_mat, max_x, max_y);
        if(nearest_food_prey.v_x != -1.0)
        {
            v4 = boids_tend_to_food(cur_prey, nearest_food_prey);
            if(run_when_hungry && cur_prey->cur_food <= (cur_prey->metabolism * steps_hungry_prey))
                v4 = v4 * running_food_param;
        }
        vect nearest_predator = get_nearest_predator(cur_prey, tile_mat, max_x, max_y);
        if(nearest_predator.v_x != -1.0)
        {
            v5 = boids_run_from_pred(cur_prey, nearest_predator);
        }
        cur_prey->velocity = cur_prey->velocity + v1 + v2 + v3 + v4 + v5;
        //limiting the speed of the agent
        if(vect_norm(cur_prey->velocity) > cur_prey->base_speed)
        {
            cur_prey->velocity =
                (cur_prey->velocity / vect_norm(cur_prey->velocity))
                * cur_prey->base_speed;
        }
        //cout<<"velocity: ("<<cur_prey->velocity.v_x<<" , "<<cur_prey->velocity.v_y<<")"<<endl;
        cur_prey->pos = cur_prey->pos + cur_prey->velocity;
        if(cur_prey->pos.v_x < 0) cur_prey->pos.v_x = max_x + cur_prey->pos.v_x;
        if(cur_prey->pos.v_x >= max_x) cur_prey->pos.v_x = cur_prey->pos.v_x - max_x;
        if(cur_prey->pos.v_y < 0) cur_prey->pos.v_y = max_y + cur_prey->pos.v_y;
        if(cur_prey->pos.v_y >= max_y) cur_prey->pos.v_y = cur_prey->pos.v_y - max_y;
        cur_prey->tile_pos->has_prey = false;
        cur_prey->tile_pos =
            tile_swap_prey(tile_mat,cur_prey,
                           &tile_mat[cur_prey->pos.v_x][cur_prey->pos.v_y],
                           max_x, max_y);
        //cout<<"prey i: "<<i<<" after pos: ("<<v_prey[i].pos.v_x<<" , "<<v_prey[i].pos.v_y<<")"<<endl;
    }
}
void predator_draw(predator* predator)
{
    int spec_ind = (predator->skin - 1) * 5 +  predator->trans;
    /*pair<float,float> tile_center = get_tile_center(predator->tile_pos);
    ALLEGRO_PATH *path = al_get_standard_path(ALLEGRO_RESOURCES_PATH);
    al_set_path_filename(path, "cheetah0101.png");
    cout<<al_path_cstr(path, '/')<<endl;

    ALLEGRO_BITMAP* pred_bitmap = al_load_bitmap(al_path_cstr(path, '/'));
    if(pred_bitmap == NULL)
    {
        cout<<"bye xdxdd"<<endl;
    }
    al_draw_bitmap(pred_bitmap, tile_center.first, tile_center.second, NULL);*/
    int v_x = predator->tile_pos->pos->v_x, v_y =  predator->tile_pos->pos->v_y;
    int s_x = v_x * tile_width, s_y = v_y * tile_height;
    al_draw_filled_triangle(s_x, s_y +  tile_height, s_x +  tile_width, s_y + tile_height,
                            s_x + (tile_width / 2), s_y, al_map_rgb(0,spec_ind * (256 / 25),0));
}

void predator_destroy(predator* predator)
{
    if(predator)
        free(predator);
}
void predator_move(vector<predator>& v_predator, vector< vector<tile> >& tile_mat,
                   int max_x, int max_y)
{
    for(int i = 0; i < v_predator.size(); i++)
    {
        predator* cur_predator = &v_predator[i];
        vect v_to_pred(0.0,0.0),v_to_mate(0.0,0.0);
        //is this predator hungry?
        if(cur_predator->cur_food < (cur_predator->metabolism * steps_hungry_predator))
        {
            //cout<<"cheetah is hungry!!"<<endl;
            //searching for the nearest prey
            vect nearest_prey =
                get_nearest_food_predator(cur_predator, tile_mat,max_x, max_y);
            //should I eat it?
            if(vect_norm(nearest_prey - cur_predator->pos) < 2)
            {
                prey* prey_to_eat = tile_mat[nearest_prey.v_x][nearest_prey.v_y].prey_on_tile;
                predator_eat_prey(cur_predator, prey_to_eat, tile_mat);
                continue;
            }
            if(nearest_prey.v_x != -1.0)
            {
                cur_predator->is_attacking = true;
                v_to_pred = tend_to_prey(cur_predator, nearest_prey);
            }

            cur_predator->velocity = cur_predator->velocity + v_to_pred;
            //limiting the speed of the agent to the max running speed
            if(vect_norm(cur_predator->velocity) > cur_predator->run_speed)
            {
                cur_predator->velocity =
                    (cur_predator->velocity / vect_norm(cur_predator->velocity))
                    * cur_predator->run_speed;
            }
            cur_predator->pos = cur_predator->pos + cur_predator->velocity;

            if(cur_predator->pos.v_x < 0) cur_predator->pos.v_x = max_x + cur_predator->pos.v_x;
            if(cur_predator->pos.v_x >= max_x) cur_predator->pos.v_x = cur_predator->pos.v_x - max_x;
            if(cur_predator->pos.v_y < 0) cur_predator->pos.v_y = max_y + cur_predator->pos.v_y;
            if(cur_predator->pos.v_y >= max_y) cur_predator->pos.v_y = cur_predator->pos.v_y - max_y;
            //cout<<"moving to: ("<<cur_predator->pos.v_x<<" , "<<cur_predator->pos.v_y<<")"<<endl;
            cur_predator->tile_pos =
                tile_swap_predator(tile_mat,cur_predator,
                                   &tile_mat[cur_predator->pos.v_x][cur_predator->pos.v_y],
                                   max_x, max_y);
        }
        //moving according to segregation rules
        else
        {
            vector<vect> pos_to_move;
            for(int j = 0; j < v_predator.size(); j++)
            {
                if(i != j)
                {
                    predator* other_pred = &v_predator[j];
                    //is the other predator in my sigth
                    if(vect_norm(cur_predator->pos - other_pred->pos) <
                                    cur_predator->vision_rad)
                    {
                        int dist = predator_get_distance(cur_predator, other_pred);
                        vect to_mate = tend_to_mate(cur_predator, other_pred->pos);
                        if(dist > affinity_threshold)
                            to_mate = to_mate * -1.0;
                        pos_to_move.push_back(to_mate);
                    }
                }
            }
            //cout<<"got pos_move vector"<<endl;
            if(pos_to_move.size() == 0)
                continue;
            for(int j = 0; j < pos_to_move.size(); j++)
                cur_predator->velocity = cur_predator->velocity + pos_to_move[j];
            //limiting the speed of the agent to the max running speed
            if(vect_norm(cur_predator->velocity) > cur_predator->base_speed)
            {
                cur_predator->velocity =
                    (cur_predator->velocity / vect_norm(cur_predator->velocity))
                    * cur_predator->base_speed;
            }
            //cout<<"move to pos move"<<endl;
            cur_predator->pos = cur_predator->pos + cur_predator->velocity;

            if(cur_predator->pos.v_x < 0) cur_predator->pos.v_x = max_x + cur_predator->pos.v_x;
            if(cur_predator->pos.v_x >= max_x) cur_predator->pos.v_x = cur_predator->pos.v_x - max_x;
            if(cur_predator->pos.v_y < 0) cur_predator->pos.v_y = max_y + cur_predator->pos.v_y;
            if(cur_predator->pos.v_y >= max_y) cur_predator->pos.v_y = cur_predator->pos.v_y - max_y;
            //cout<<"swapping to tile ("<<cur_predator->pos.v_x<<" , "<<cur_predator->pos.v_y<<")"<<endl;
            cur_predator->tile_pos =
                tile_swap_predator(tile_mat,cur_predator,
                                   &tile_mat[cur_predator->pos.v_x][cur_predator->pos.v_y],
                                   max_x, max_y);
            //cout<<"tile swapped"<<endl;
        }
    }
}
void predator_reproduce(vector<predator>& v_predator, vector< vector<tile> >&tile_mat,
                        int max_x, int max_y)
{
    for(int i = 0; i < v_predator.size(); i++)
    {
        predator* cur_predator = &v_predator[i];
        int min_dist = -1, min_ind = -1;
        if(cur_predator->cur_age > adult_age_predator /*&&
            cur_predator->cur_food >= cur_prey->capacity / 2*/)
        {
            for(int j = 0; j < v_predator.size(); j++)
            {
                if(j != 1)
                {
                    predator* other_predator = &v_predator[j];
                    if(cur_predator->is_male^other_predator->is_male)
                        cout<<"found male and female"<<endl;
                    if(cur_predator->is_male^other_predator->is_male &&
                            vect_norm(cur_predator->pos - other_predator->pos) < dist_to_reproduce_predator)
                    {
                        int spec_dist = predator_get_distance(cur_predator, other_predator);
                        if(min_dist == -1)
                        {
                            min_dist = spec_dist;
                            min_ind = j;
                            continue;
                        }
                        if(spec_dist < min_dist)
                        {
                            min_dist = spec_dist;
                            min_ind = j;
                        }
                    }
                }
            }
            //found some lover :p
            if(min_dist != -1)
            {
                cout<<"cheetah found lover"<<endl;
                predator* lover = &v_predator[min_ind];
                float prob = min_dist / 10;

                double r = rand() % RAND_MAX;
                if(r > prob)
                {
                    int tries = 0;
                    vect middle = (cur_predator->pos + lover->pos) / 2.0;
                    int skin = (cur_predator->skin + lover->skin) / 2;
                    int trans = (cur_predator->trans + lover->trans) / 2;
                    bool success =
                        predator_child_born(v_predator, tile_mat, middle, skin, trans,
                                        max_x, max_y);
                    if(success)
                    {
                        cout<<"child cheetah was born at: ("<<middle.v_x<<" , "<<middle.v_y<<")"<<endl;
                    }
                }
            }
        }
    }
}
bool predator_child_born(vector<predator>& v_predator, vector< vector<tile> >& tile_mat,
                         vect pos, int skin, int trans, int max_x, int max_y)
{
    bool success = true;
    int tries = 0;
    tile* next_tile = &tile_mat[pos.v_x][pos.v_y];
    while((next_tile->has_prey || next_tile->has_predator))
    {
        if(tries == 1)
        {
            success =false;
            break;
        }
        double n_rand = ((double) rand() / (RAND_MAX));
        int to_add = rand() % 3;
        if(n_rand > 0.5)
        {
            if(next_tile->pos->v_x + to_add < max_x)
            {
                next_tile = &tile_mat[next_tile->pos->v_x + to_add][next_tile->pos->v_y];

            }
            else
            {
                next_tile = &tile_mat[next_tile->pos->v_x - to_add][next_tile->pos->v_y];
            }
        }
        else
        {
            if(next_tile->pos->v_y + to_add < max_y)
            {

                next_tile = &tile_mat[next_tile->pos->v_x][next_tile->pos->v_y + to_add];
            }
            else
            {
                next_tile = &tile_mat[next_tile->pos->v_x][next_tile->pos->v_y - to_add];
            }
        }
        tries++;
    }
    if(success)
    {
        v_predator.push_back(*predator_init(next_tile, skin, trans));
    }
    return success;
}
int predator_get_distance(predator* cur_predator, predator* other_predator)
{
    return abs(cur_predator->skin - other_predator->skin) +
           abs(cur_predator->trans - other_predator->trans);
}
void predator_eat_prey(predator* predator, prey* prey, vector< vector<tile> >& tile_mat)
{
    predator->is_attacking = false;
    predator->cur_food = predator->capacity;
    prey->has_been_eaten = true;
}
float get_radians(float degrees)
{
    const double halfC = M_PI / 180;
    return degrees * halfC;
}
point* next_point(point* p, float angle, float length)
{
    point* new_point = (point*)malloc(sizeof(point));
    //is the angle in the first quadrant?
    if (angle >=0 && angle <= 90 )
    {
        new_point->x = p->x - length*cos(get_radians(angle));
        new_point->y = p->y - length*sin(get_radians(angle));
    }
    //is the angle in the second quadrant?
    if (angle > 90 && angle <= 180)
    {
        float r_angle = 180.0 - angle;
        new_point->x = p->x + length*cos(get_radians(r_angle));
        new_point->y = p->y - length*sin(get_radians(r_angle));
    }
    //is the angle in the third quadrant?
    if (angle > 180 && angle <= 270)
    {
        float r_angle = 270.0 - angle;
        new_point->x = p->x + length*sin(get_radians(r_angle));
        new_point->y = p->y + length*cos(get_radians(r_angle));
    }
    //is the angle in the fourth quadrant?
    if (angle > 270 && angle <= 360)
    {
        float r_angle = 360.0 - angle;
        new_point->x = p->x - length*cos(get_radians(r_angle));
        new_point->y = p->y + length*sin(get_radians(r_angle));
    }
    if(new_point->x < 0)
        new_point->x = WIDTH + new_point->x;
    if(new_point->x > WIDTH)
        new_point->x = (new_point->x - WIDTH);
    if(new_point->y < 0)
        new_point->y = HEIGHT + new_point->y;
    if(new_point->y > HEIGHT)
        new_point->y = (new_point->y - HEIGHT);
    return new_point;
}
void add_sand(vector< vector<tile> >& tile_mat, int i,
              int j, int n, int m, int max_food, int to_add)
{
    //cout<<"to_add: "<<to_add<<endl;
    tile_mat[i][j].food += to_add;
    //cout<<"tile_mat["<<i<<"]["<<j<<"]: "<<tile_mat[i][j].food<<endl;
    if (tile_mat[i][j].food >= max_food)
    {
        int next_add = (tile_mat[i][j].food - 1) / 4;
        tile_mat[i][j].food = 1;
        if (i + 1 < n)
        {
            add_sand(tile_mat, i+1 , j , n, m,
                     max_food, next_add);
        }
        if( j + 1 < m)
        {
            add_sand(tile_mat, i , j+1 , n, m,
                     max_food, next_add);
        }
        if (i - 1 >= 0)
        {
            add_sand(tile_mat, i-1 , j , n, m,
                     max_food, next_add);
        }
        if( j - 1 >= 0)
        {
            add_sand(tile_mat, i , j-1 , n, m,
                     max_food, next_add);
        }

    }
}
int get_tile_distance(tile* tile1, tile* tile2, int max_x, int max_y)
{
    return min(abs(tile2->pos->v_x - tile1->pos->v_x),((WIDTH / tile_width) -
               tile1->pos->v_x) + tile2->pos->v_x) +
           min(abs(tile2->pos->v_y - tile1->pos->v_y),((HEIGHT / tile_height) -
                   tile1->pos->v_y) + tile2->pos->v_y);
}
pair<float,float> get_tile_center(tile* tile)
{
    //cout<<"getting center for tile: "<<tile->pos->v_x<<" , "<<tile->pos->v_y<<endl;
    return make_pair(tile->pos->v_x * tile_width + (tile_width / 2.0),
                     tile->pos->v_y * tile_height + (tile_height / 2.0));
}
tile* get_tile_from_coord(int x, int y, vector< vector<tile> >& tile_mat)
{
    return &tile_mat[x / tile_width][y / tile_height];
}
tile* tile_swap_prey(vector< vector<tile> >& tile_mat,
                     prey* prey, tile* next_tile, int max_x, int max_y)
{
    tile_mat[prey->tile_pos->pos->v_x][prey->tile_pos->pos->v_y].has_prey = false;
    int c = 0;
    while(next_tile->has_prey || next_tile->has_predator)
    {
        //cout<<"swapping prey: ("<<next_tile->pos->v_x<<" , "<<next_tile->pos->v_y<<")"<<endl;
        //breaking deadlocks
        if(c==20)
        {
            //cout<<"got to break"<<endl;
            break;
        }
        //cout<<"trying tile: ("<<next_tile->pos->v_x<<" , "<<next_tile->pos->v_y<<")"<<endl;
        double n_rand = ((double) rand() / (RAND_MAX));
        int to_add = rand() % 3;
        if(n_rand > 0.5)
        {
            if(next_tile->pos->v_x + to_add < max_x)
            {
                next_tile = &tile_mat[next_tile->pos->v_x + to_add][next_tile->pos->v_y];

            }
            else
            {
                next_tile = &tile_mat[next_tile->pos->v_x - to_add][next_tile->pos->v_y];
            }
        }
        else
        {
            if(next_tile->pos->v_y + to_add < max_y)
            {

                next_tile = &tile_mat[next_tile->pos->v_x][next_tile->pos->v_y + to_add];
            }
            else
            {
                next_tile = &tile_mat[next_tile->pos->v_x][next_tile->pos->v_y - to_add];
            }
        }
        c++;
    }
    next_tile->has_prey = true;
    next_tile->prey_on_tile = prey;
    return next_tile;
}
tile* tile_swap_predator(vector< vector<tile> >& tile_mat,
                         predator* predator, tile* next_tile, int max_x, int max_y)
{
    tile_mat[predator->tile_pos->pos->v_x][predator->tile_pos->pos->v_y].has_prey = false;
    int c = 0;
    while(next_tile->has_prey || next_tile->has_predator)
    {
        //cout<<"swapping predator: ("<<next_tile->pos->v_x<<" , "<<next_tile->pos->v_y<<")"<<endl;
        //breaking deadlocks
        if(c == 20)
        {
            //cout<<"got to break"<<endl;
            break;
        }

        //cout<<"trying tile pred: ("<<next_tile->pos->v_x<<" , "<<next_tile->pos->v_y<<")"<<endl;
        double n_rand = ((double) rand() / (RAND_MAX));
        int to_add = rand() % 3;
        if(n_rand > 0.5)
        {
            if(next_tile->pos->v_x + to_add < max_x)
            {
                next_tile = &tile_mat[next_tile->pos->v_x + to_add][next_tile->pos->v_y];

            }
            else
            {
                next_tile = &tile_mat[next_tile->pos->v_x - to_add][next_tile->pos->v_y];
            }
        }
        else
        {
            if(next_tile->pos->v_y + to_add < max_y)
            {

                next_tile = &tile_mat[next_tile->pos->v_x][next_tile->pos->v_y + to_add];
            }
            else
            {
                next_tile = &tile_mat[next_tile->pos->v_x][next_tile->pos->v_y - to_add];
            }
        }
        c++;
    }
    next_tile->has_predator = true;
    next_tile->pred_on_tile = predator;
    return next_tile;
}
int power_law_val(int param, int x0, int x1)
{
    double n_uniform = rand()/ double(RAND_MAX);
    double val = pow(((pow(x1, param+1) - pow(x0, param+1))*n_uniform +
                      pow(x0, param +1)) , (1.0/(param+1.0)));
    //cout<<"uniform: "<<n_uniform<<endl;
    //cout<<"val: "<<val<<endl;
    return (int)val;
}
vect boids_rule1(vector<prey>& v_prey, int cur_prey_ind)
{
    vect pcj(0.0,0.0);
    //bool in_boid = false;
    //TODO: Taking into account the vision of the agent
    for(int i = 0; i < v_prey.size(); i++)
    {
        if(i != cur_prey_ind &&
                vect_norm(*v_prey[cur_prey_ind].tile_pos->pos -
                          *v_prey[i].tile_pos->pos) <= v_prey[i].vision_rad)
        {
            //in_boid = true;
            pcj = pcj + *v_prey[i].tile_pos->pos;
        }
    }
    /*if (in_boid)
    {*/
    pcj = pcj / (v_prey.size() - 1);
    return (pcj - *v_prey[cur_prey_ind].tile_pos->pos) / boid1_param;
    /*}
    else
    {
        /*walk in random walk
        pcj.v_x = rand() % 10 - 5;
        pcj.v_y = rand() % 10 - 5;
        return pcj / 100.0;
    }*/
}
vect boids_rule2(vector<prey>& v_prey, int cur_prey_ind)
{
    vect c(0.0,0.0);
    for(int i = 0; i < v_prey.size(); i++)
    {
        if(i != cur_prey_ind)
        {
            if (vect_norm(*v_prey[cur_prey_ind].tile_pos->pos -
                          *v_prey[i].tile_pos->pos) < 10)
            {
                c = c - (*v_prey[cur_prey_ind].tile_pos->pos -
                         *v_prey[i].tile_pos->pos);
            }
        }
    }
    return c / boid2_param;
}
vect boids_rule3(vector<prey>& v_prey, int cur_prey_ind)
{
    vect pvj(0.0,0.0);
    //bool in_boid = false;
    //TODO: Taking into account the vision of the agent
    for(int i = 0; i < v_prey.size(); i++)
    {
        if(i != cur_prey_ind &&
                vect_norm(*v_prey[cur_prey_ind].tile_pos->pos -
                          *v_prey[i].tile_pos->pos) <= v_prey[i].vision_rad)
        {
            //in_boid = true;
            pvj = pvj + v_prey[i].velocity;
        }
    }
    /*if (in_boid)
    {*/
    pvj = pvj / (v_prey.size() - 1);
    return (pvj - v_prey[cur_prey_ind].velocity) / boid3_param;
    /*}
    else
    {
        /*walk in random walk
        pvj.v_x = rand() % 10 - 5;
        pvj.v_y = rand() % 10 - 5;
        return pvj / 100.0;
    }*/
}
vect boids_tend_to_food(prey* prey, vect place)
{
    return (place - prey->pos) / tend_food_param;
}
vect boids_run_from_pred(prey* prey, vect place)
{
    return (place - prey->pos) * run_from_pred_param;
}
vect tend_to_prey(predator* predator, vect prey_tile)
{
    return (prey_tile - predator->pos) * tend_prey_param;
}
vect tend_to_mate(predator* predator, vect mate_tile)
{
    return (mate_tile - predator->pos) / tend_mate_param;
}
vect get_nearest_predator(prey* prey, vector< vector<tile> >& tile_mat,
                          int max_x, int max_y)
{
    float min_dist = -1;
    vect to_return(-1.0,-1.0);
    for(int i = 0; i < max_x; i++)
    {
        for(int j = 0; j < max_y; j++)
        {
            float cur_dist = vect_norm(prey->pos - *tile_mat[i][j].pos);
            if(tile_mat[i][j].has_predator && cur_dist < (prey->vision_rad)
                && tile_mat[i][j].pred_on_tile->is_attacking)
            {
                if(min_dist == -1)
                {
                    min_dist = cur_dist;
                    to_return = *tile_mat[i][j].pos;
                    continue;
                }
                if(cur_dist < min_dist)
                {
                    min_dist = cur_dist;
                    to_return = *tile_mat[i][j].pos;
                }
            }
        }
    }
    return to_return;
}
vect get_nearest_food_prey(prey* prey, vector< vector<tile> >& tile_mat,
                           int max_x, int max_y)
{
    float min_dist = -1;
    vect to_return(-1.0,-1.0);
    for(int i = 0; i < max_x; i++)
    {
        for(int j = 0; j < max_y; j++)
        {
            float cur_dist = vect_norm(prey->pos - *tile_mat[i][j].pos);
            if(!tile_mat[i][j].has_prey && !tile_mat[i][j].has_predator
                    && tile_mat[i][j].food > 0 && cur_dist < prey->vision_rad)
            {
                if(min_dist == -1)
                {
                    min_dist = cur_dist;
                    to_return = *tile_mat[i][j].pos;
                    continue;
                }
                if(cur_dist < min_dist)
                {
                    min_dist = cur_dist;
                    to_return = *tile_mat[i][j].pos;
                }
            }
        }
    }
    return to_return;
}
vect get_nearest_food_predator(predator* predator, vector< vector<tile> >& tile_mat,
                               int max_x, int max_y)
{
    float min_dist = -1;
    vect to_return(-1.0,-1.0);
    for(int i = 0; i < max_x; i++)
    {
        for(int j = 0; j < max_y; j++)
        {
            float cur_dist = vect_norm(predator->pos - *tile_mat[i][j].pos);
            if(tile_mat[i][j].has_prey && cur_dist < predator->vision_rad)
            {
                if(min_dist == -1)
                {
                    min_dist = cur_dist;
                    to_return = *tile_mat[i][j].pos;
                    continue;
                }
                if(cur_dist < min_dist)
                {
                    min_dist = cur_dist;
                    to_return = *tile_mat[i][j].pos;
                }
            }
        }
    }
    return to_return;
}
vect get_nearest_mate(predator* predator, vector< vector<tile> >& tile_mat,
                        int max_x, int max_y)
{
    float min_dist = -1.0;
    vect to_return(-1.0,-1.0);
    for(int i = 0; i < max_x; i++)
    {
        for(int j = 0; j < max_y; j++)
        {
//            if()
        }
    }
}
float vect_norm(vect v)
{
    return sqrt(pow(v.v_x,2.0) + pow(v.v_y, 2.0));
}


