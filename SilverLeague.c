
// Includes
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

// Defines
#define CP_MAX_COUNT        10
#define CP_RADIUS           400

// Macros
#define OUTPUT(x, y, str)   printf("%d %d %s\n", x, y, str);

// Struct
typedef struct {
    int         x;
    int         y;
} Point;
typedef struct{
    Point       cp_positions[CP_MAX_COUNT];
    int         cp_distances[CP_MAX_COUNT];
    int         cp_count;
    int         current_cp_idx;
    int         current_lap;
} CPManager;
typedef struct{
    Point       position;
    Point       next_cp;
    Point       opponent;
    Point       last_pod_point;
    Point       speed_point;
    Point       setpoint;
    CPManager   cp_manager;
    int         next_cp_dist;
    int         next_cp_angle;
    int         previous_opponent_dist;
    int         abs_speed;
    int         setpoint_angle;
    int         thrust;
    bool        shield;
    bool        shield_cooldown; 
    bool        boost;
    bool        boost_used;
    int         best_boost_cp_idx;
} POD;

// Functions prototypes
void ExistPoint(Point *p, int size, Point new, bool *res, int *idx);
void CalculateDistance(Point point1, Point point2, float *distance);
void CalculateAngle(Point point1, Point point2, float *angle);
void NormalizeAngle(float *angle);
//
void CPManager_Init(CPManager *manager);
void POD_Init(POD *pod);
//
void BestBoostCP(CPManager *manager, int *idx);
void CPManagement(POD *pod, CPManager *manager);
//
void SetpointControll(POD *pod, int *setpoint_x, int *setpoint_y);
void ThrustControll(POD *pod, int *thrust);
void BoostControll(POD *pod, bool *boost);
void ShieldControll(POD* pod, bool *shield);
void PODController(POD *pod);

int main()
{
    // Write an action using printf(). DON'T FORGET THE TRAILING \n
    // To debug: fprintf(stderr, "Debug messages...\n");
    // You have to output the target position
    // followed by the power (0 <= thrust <= 100) or "BOOST" or "SHIELD"
    // i.e.: "x y thrust"
    
    POD jaquim = {0};
    char num_str[3] = {0};

    CPManager_Init(&jaquim.cp_manager);
    POD_Init(&jaquim);

    while (1)
    {
        scanf("%d%d%d%d%d%d", &jaquim.position.x, &jaquim.position.y, &jaquim.next_cp.x, &jaquim.next_cp.y, &jaquim.next_cp_dist, &jaquim.next_cp_angle);
        scanf("%d%d", &jaquim.opponent.x, &jaquim.opponent.y);

        CPManagement(&jaquim, &jaquim.cp_manager);
       
        // SPEED calculation
        jaquim.speed_point.x = (jaquim.position.x - jaquim.last_pod_point.x);
        jaquim.speed_point.y = (jaquim.position.y - jaquim.last_pod_point.y);
        jaquim.last_pod_point = jaquim.position;
        jaquim.abs_speed = sqrt((jaquim.speed_point.x * jaquim.speed_point.x) + (jaquim.speed_point.y * jaquim.speed_point.y));

        // ROTATION Controll
        SetpointControll(&jaquim, &jaquim.setpoint.x, &jaquim.setpoint.y);

        // THRUST Controll
        ThrustControll(&jaquim, &jaquim.thrust);

        // BOOST Controll
        BoostControll(&jaquim, &jaquim.boost);

        // SHIELD Controll 
        ShieldControll(&jaquim, &jaquim.shield);

        //OUTPUT
        if (jaquim.boost)
        {
            OUTPUT(jaquim.setpoint.x, jaquim.setpoint.y, "BOOST");
        }
        else if (jaquim.shield)
        {
            OUTPUT(jaquim.setpoint.x, jaquim.setpoint.y, "SHIELD");
        }
        else
        {
            sprintf(num_str, "%d", jaquim.thrust); 
            OUTPUT(jaquim.setpoint.x, jaquim.setpoint.y, num_str);
        }
    }

    return 0;
}

// Functions
void CPManager_Init(CPManager *manager)
{
    manager->cp_count = 0;
    memset(manager->cp_positions, 0, CP_MAX_COUNT);
    memset(manager->cp_distances, 0, CP_MAX_COUNT);
    manager->current_cp_idx = 0;
    manager->current_lap = 0;
}
void POD_Init(POD *pod)
{
    pod->last_pod_point.x = 0;
    pod->last_pod_point.y = 0;
    pod->shield = false;
    pod->shield_cooldown = true;
    pod->boost_used = false;
    pod->boost = false;
    pod->best_boost_cp_idx = 100;
}
void ExistPoint(Point *p, int size, Point new, bool *res, int *idx) 
{
    int i = 0;
    for (; i < size; i++) 
    {
        if (p[i].x == new.x && p[i].y == new.y) 
        {
            (*res) = true;
            (*idx) = i;
            return; // A coordenada já existe
        }
    }
    
    (*idx) = i;
    (*res) = false; // A coordenada não existe
}
void BestBoostCP(CPManager *manager, int *cp_idx) 
{
    int best_checkpoint_idx = 0;
    int maxDistance = 0;

    for (int idx = 1; idx < manager->cp_count; idx++) 
    {
        // Obtém a distância entre dois checkpoints consecutivos
        if (manager->cp_distances[idx - 1] > maxDistance) 
        {
            maxDistance = manager->cp_distances[idx - 1];
            best_checkpoint_idx = idx - 1;
        }
    }

    (*cp_idx) = best_checkpoint_idx;
    //fprintf(stderr, "melhor boost idx: %d\n",  *cp_idx);
    //fprintf(stderr, "melhor boost distance: %d\n",  maxDistance);
}
void CPManagement(POD *pod, CPManager *manager)
{
    static Point last_cp = {0};
    int cp_idx = 0; 
    bool diff_cp = false;
    bool exist_cp = false;

    if (manager->cp_count < CP_MAX_COUNT) 
    {
        if (manager->cp_count == 0)
        {
            diff_cp = true;
            exist_cp = false;
            cp_idx = 0;
        }
        else if (last_cp.x != pod->next_cp.x && last_cp.y != pod->next_cp.y)
        {
            diff_cp = true;
            ExistPoint(manager->cp_positions, manager->cp_count, pod->next_cp, &exist_cp, &cp_idx);
        }
        else
        {
            diff_cp = false;
        }
        last_cp = pod->next_cp;
    }

    if (manager->cp_count > 1 && cp_idx == 0 && diff_cp && exist_cp)
    {
        manager->current_lap++;
    }

    if (manager->current_lap == 0 && diff_cp && !exist_cp)
    {
        manager->cp_positions[cp_idx].x = pod->next_cp.x;
        manager->cp_positions[cp_idx].y = pod->next_cp.y;
        manager->cp_distances[cp_idx] = pod->next_cp_dist;
        //fprintf(stderr, "Novo CP: %d\n",  cp_idx);
        BestBoostCP(manager, &pod->best_boost_cp_idx);
        manager->cp_count++;
    }

    // atualizar indice do checkpoint atual
    if (diff_cp && exist_cp)
    {
        manager->current_cp_idx = cp_idx;
    }

    //fprintf(stderr, "Volta: %d\n", manager->current_lap);
    //fprintf(stderr, "Num CP: %d\n",  manager->checkpoint_count);
}
void CalculateDistance(Point point1, Point point2, float *distance)
{
    float dx = point2.x - point1.x;
    float dy = point2.y - point1.y;
    (*distance) = sqrt(dx * dx + dy * dy);
}
void CalculateAngle(Point point1, Point point2, float *angle) 
{
    float dx = point2.x - point1.x;
    float dy = point2.y - point1.y;
    (*angle) = atan2(dy, dx) * 180.0 / M_PI;
}
void NormalizeAngle(float *angle) 
{
    if (*angle < 0) 
    {
        *angle += 360.0;
    }
    
    if (*angle >= 360.0) 
    {
        *angle -= 360.0;
    }
}
void SetpointControll(POD *pod, int *setpoint_x, int *setpoint_y)
{
    float distance_th = 3.5;
    distance_th = pod->abs_speed * distance_th;
    fprintf(stderr, "speed: %d\n", pod->abs_speed);
    fprintf(stderr, "distance_th: %.1f\n", distance_th);

    int lateral_offset          = 0;
    int vertical_offset         = 0;
    //
    int next_next_cp_idx        = 0;
    Point next_next_cp_pos      = {0};
    float next_next_cp_angle    = 0;
    float drift_radius          = 300;
    Point target_vector         = {0};
    float target_magnitude      = 0;
    Point perpendicular_vector  = {0};

    if (pod->speed_point.x > 0)
    {
        // movimenta-se para a direita
        lateral_offset = (-CP_RADIUS);
    }
    else if (pod->speed_point.x < 0)
    {
       // movimenta-se para a esquerda
       lateral_offset = (CP_RADIUS);
    }

    if (pod->speed_point.y > 0)
    {
        // movimenta-se para baixo
        vertical_offset = (-CP_RADIUS);
    }
    else if (pod->speed_point.y < 0)
    {
        // movimenta-se para cima
        vertical_offset = (CP_RADIUS);
    }

    if (pod->next_cp_dist <= distance_th && pod->cp_manager.current_lap > 0) 
    {
        next_next_cp_idx = (pod->cp_manager.current_cp_idx + 1) % pod->cp_manager.cp_count;// "Overflow" protection
        next_next_cp_pos.x = pod->cp_manager.cp_positions[next_next_cp_idx].x;
        next_next_cp_pos.y = pod->cp_manager.cp_positions[next_next_cp_idx].y;

        CalculateAngle(pod->position, next_next_cp_pos, &next_next_cp_angle);
        NormalizeAngle(&next_next_cp_angle);
        pod->setpoint_angle = next_next_cp_angle;//PROBLEMAS COM ESTE ANGULO

        drift_radius = 200;
        drift_radius = drift_radius / next_next_cp_angle;

        target_vector.x = next_next_cp_pos.x - pod->position.x;
        target_vector.y = next_next_cp_pos.y - pod->position.y;

        target_magnitude = sqrt(target_vector.x * target_vector.x + target_vector.y * target_vector.y);
        target_vector.x /= target_magnitude;
        target_vector.y /= target_magnitude;

        perpendicular_vector.x = -target_vector.y;
        perpendicular_vector.y = target_vector.x;

        fprintf(stderr, "next_next_cp_angle: %.1f\n", next_next_cp_angle);
        fprintf(stderr, "drift_radius: %.1f\n", drift_radius);
        //fprintf(stderr, "target_vector_x: %d\n", target_vector.x);
        //fprintf(stderr, "target_vector_y: %d\n", target_vector.y);
        //fprintf(stderr, "target_magnitude: %.1f\n", target_magnitude);
        //fprintf(stderr, "perpendicular_vector_x: %d\n", perpendicular_vector.x);
        //fprintf(stderr, "perpendicular_vector_y: %d\n", perpendicular_vector.y);

        (*setpoint_x) = next_next_cp_pos.x + drift_radius * perpendicular_vector.x + (lateral_offset*2);
        (*setpoint_y) = next_next_cp_pos.y + drift_radius * perpendicular_vector.y + (vertical_offset*2);
    }
    else
    {
        pod->setpoint_angle = pod->next_cp_angle;
        (*setpoint_x) = pod->next_cp.x + lateral_offset;
        (*setpoint_y) = pod->next_cp.y + vertical_offset;
    }
}
void ThrustControll(POD *pod, int *thrust)
{
    const int max_thrust            = 100;

    int epsilon                     = 40;// threshold angulo para começar o ajuste
    float k                         = 0;// ajuste thrust pela distancia do cp
    int setpoint_angle              = 0;
    int setpoint_distance           = 0;
    float thrust_distance_ratio     = 0;
    float thrust_angle_ratio        = 0;

    if (pod->cp_manager.current_lap == 0)
    {
        setpoint_angle = abs(pod->next_cp_angle);
        setpoint_distance = pod->next_cp_dist;
        epsilon = 40;
        k = 1;
    }
    else
    {
        setpoint_angle = abs(pod->setpoint_angle);
        setpoint_distance = pod->next_cp_dist;
        epsilon = 90;
        k = 0.5;
    }

    //fprintf(stderr, "setpoint_angle: %d ", setpoint_angle);
    if (setpoint_angle < epsilon)
    {
        (*thrust) = max_thrust;
        //fprintf(stderr, "full power\n");
    }
    else
    {
        // Quanto mais desalinhado estamos, mais reduzimos a velocidade
        thrust_angle_ratio = fmax(0, 1 - (double)setpoint_angle / 90);

        // Reduzir a velocidade à medida que nos aproximamos do checkpoint
        thrust_distance_ratio = fmax(0, fmin((double)setpoint_distance / (k * CP_RADIUS), 1));  
        
        (*thrust) = max_thrust * thrust_angle_ratio * thrust_distance_ratio;
        //fprintf(stderr, "\nthrust_angle_ratio: %.1f\n", thrust_angle_ratio);
        //fprintf(stderr, "thrust_distance_ratio: %.1f\n", thrust_distance_ratio);
    }
}
void BoostControll(POD *pod, bool *boost)
{
    if (!pod->boost_used && pod->cp_manager.current_cp_idx == pod->best_boost_cp_idx && 
    pod->cp_manager.current_lap > 0 && pod->setpoint_angle < 10)
    {
        pod->boost_used = true;
        (*boost) = true;
    }
    else
    {
        (*boost) = false;
    }
}
void ShieldControll(POD* pod, bool *shield)
{
    const int scale = 1000;
    const int collision_threshold = 100;

    static bool enable_cooldown_counter = false;
    static int cooldown_counter = 0;

    float opponent_distance = 0;
    float opponent_approaching_rate = 0; 
    float collision_value = 0;

    // não aplicar shield quando oponente esta a minha frente em relação ao proximo cp
    opponent_distance = sqrt(pow(pod->position.x - pod->opponent.x, 2) + pow(pod->position.y - pod->opponent.y, 2));
    opponent_approaching_rate = pod->previous_opponent_dist - opponent_distance;
    pod->previous_opponent_dist = opponent_distance;
    collision_value = scale * (1.0 / opponent_distance) * opponent_approaching_rate;

    if (collision_value >= collision_threshold && opponent_distance < 1000 && pod->shield_cooldown)
    {
        (*shield) = true;
        enable_cooldown_counter = true;
    }
    else
    {
        (*shield) = false;
    }

    if (enable_cooldown_counter)
    {
        if (cooldown_counter >= 3)
        {
            cooldown_counter = 0;
            pod->shield_cooldown = true;
            enable_cooldown_counter = false;
        }
        else
        {
            cooldown_counter++;
            pod->shield_cooldown = false;
        }
    }

    //fprintf(stderr, "distance: %.1f\n", opponent_distance);
    //fprintf(stderr, "taxa apro: %.1f\n", opponent_approaching_rate);
    //fprintf(stderr, "collision_value: %.1f\n", collision_value);
    //fprintf(stderr, "enable_cooldown_counter: %d\n", enable_cooldown_counter);
    //fprintf(stderr, "cooldown: %d\n", pod->shield_cooldown);
    //fprintf(stderr, "cooldown_counter: %d\n", cooldown_counter);
}

// END
