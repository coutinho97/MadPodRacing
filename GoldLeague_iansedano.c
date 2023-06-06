#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <math.h>
#include <string>
#include <vector>

using namespace std;

constexpr int   SIMULATIONTURNS     = 4;// number of turns to simulate per solution
constexpr int   SOLUTIONCOUNT       = 6;// size of the population (genetic evolution)
constexpr int   maxThrust           = 100;
constexpr int   boostThrust         = 650;
constexpr int   maxRotation         = 18;
constexpr int   shieldCooldown      = 4;// shield turn + 3
constexpr int   timeoutFirstTurn    = 500;//ms
constexpr int   timeout             = 75;//ms
constexpr float checkpointRadius    = 600.f;
constexpr float checkpointRadiusSqr = checkpointRadius * checkpointRadius;
constexpr float podRadius           = 400.f;
constexpr float podRadiusSqr        = podRadius * podRadius;
constexpr float minImpulse          = 120.f;
constexpr float frictionFactor      = .85f;
constexpr float PI                  = 3.14159f;

// https://software.intel.com/en-us/articles/fast-random-number-generator-on-the-intel-pentiumr-4-processor, apparently much faster than std::rand
inline int fastrand() { static unsigned int g_seed = 100; g_seed = (214013*g_seed+2531011); return (g_seed>>16)&0x7FFF;}
inline int rnd(int a, int b) {return (fastrand() % (b-a)) + a;}

// struct
struct Move
{
    int rotation = 0; // -maxRotation, maxRotation
    int thrust = 0; // 0, maxThrust
    bool shield = false;
    bool boost = false;
};
struct myVector
{
    // basic 2D vector to simplify notations
    float x;
    float y;

    myVector(float _x, float _y) 
    {
        x = _x;
        y = _y;
    }
    myVector()
    {
        x = 0.f;
        y = 0.f;
    }

    myVector normalized() const
    {
        const float norm = sqrt(x*x+y*y);
        return myVector(x/norm, y/norm);
    }
};
struct Pod
{
    //int id = -1; // debug only
    myVector position;
    myVector speed;
    int      angle              = -1;
    int      checkpointId       = 0;
    int      checkpointsPassed  = 0;
    bool     canBoost           = true;
    int      shieldCd           = 0;
    int      score              = 0;
};

//functions prototypes
bool operator==(const myVector& a, const myVector& b);
myVector operator+(const myVector& a, const myVector& b);
myVector operator+=(myVector& a, const myVector& b);
myVector operator-(const myVector& a, const myVector& b);
myVector operator*(float k, const myVector& a);
myVector operator*=(myVector& a, float k);
ostream& operator<<(ostream& os, const myVector& v);
int Mass(const Pod& p);
float distSqr(const myVector& a, const myVector& b);
float dist(const myVector& a, const myVector& b);
float dot(const myVector& a, const myVector& b);
float CollisionTime(Pod& p1, Pod& p2);
void ManageShield(bool turnOn, Pod& p);
void Rebound(Pod& a, Pod& b);
void PrintPod(const Pod& p);
void UpdatePodFromInput(Pod& p, int id);
void OverrideAngle(Pod& p, const myVector& target);

// class
class Turn
{
    // the moves of both pods during a turn
    private:
        vector<Move> moves = vector<Move>(2);
    public:
        Move& operator[](size_t m)             { return moves[m]; } //0<=m<2
        const Move& operator[](size_t m) const { return moves[m]; } //0<=m<2
};
class Solution
{
    // all the moves to perform the next SIMULATIONTURNS turns
    private:
        vector<Turn> turns = vector<Turn>(SIMULATIONTURNS);
    public:
        Turn& operator[](size_t t)             { return turns[t]; } //0<=t<SIMULATIONTURNS
        const Turn& operator[](size_t t) const { return turns[t]; } //0<=t<SIMULATIONTURNS

        int score = -1;
};
class Simulation
{
    private:
        int laps;
        vector<myVector> checkpoints;
        int cp_count;
        int cp_max;

    public:
        int MaxCheckpoints() const { return cp_max; }
        const vector<myVector>& Checkpoints() const { return checkpoints; }

        myVector InitCheckpointsFromInput()
        {
            cin >> laps; cin.ignore();
            cin >> cp_count; cin.ignore();

            checkpoints.reserve(cp_count);
            for (int i = 0; i < cp_count; i++)
            {
                int checkpointX, checkpointY;
                cin >> checkpointX >> checkpointY; cin.ignore();
                checkpoints[i] = myVector(checkpointX, checkpointY);
            }

            cp_max = laps*cp_count;

            // return the first checkpoint (that the pods will face from the start)
            return checkpoints[1]; // checkpoints.size() > 1
        }
        void PlaySolution(vector<Pod>& pods, const Solution& s) const
        {
            for (int i=0; i<SIMULATIONTURNS; i++)
            {
                PlayOneTurn(pods, s[i]);
            }
        }

    private:
        void PlayOneTurn(vector<Pod>& pods, const Turn& turn) const
        {
            // those are the steps described in "expert rules", in order
            Rotate(pods, turn);
            Accelerate(pods, turn);
            UpdatePositions(pods);
            Friction(pods);
            EndTurn(pods);
        }
        void Rotate(vector<Pod>& pods, const Turn& turn) const
        {
            for (int i=0; i<2; i++)
            {
                Pod& p = pods[i];
                const Move& m = turn[i];
                p.angle = (p.angle + m.rotation) % 360;
            }
        }
        void Accelerate(vector<Pod>& pods, const Turn& turn) const
        {
            for (int i=0; i<2; i++)
            {
                Pod& p = pods[i];
                const Move& m = turn[i];

                // shield
                ManageShield(m.shield, p);
                if (p.shieldCd > 0)
                    continue; // no thrust this turn

                const float angleRad = p.angle * PI / 180.f;
                const myVector direction{cos(angleRad), sin(angleRad)};

                // boost
                const bool useBoost = p.canBoost && m.boost;
                const int thrust = useBoost ? boostThrust : m.thrust;
                if (useBoost)
                    p.canBoost = false;

                p.speed += thrust * direction;
            }
        }
        void UpdatePositions(vector<Pod>& pods) const
        {
            float t = 0.f;
            constexpr float turnEnd = 1.f;
            while (t < turnEnd)
            {
                // first pods to collide with each other
                Pod* a = nullptr;
                Pod* b = nullptr;
                float dt = turnEnd - t; // date of the collision (from t)

                for (int i=0; i<4; i++)
                {
                    for (int j=i+1; j<4; j++)
                    {
                        const float collisionTime = CollisionTime(pods[i], pods[j]);
                        if ( (t+collisionTime < turnEnd) && (collisionTime < dt) )
                        {
                            dt = collisionTime;
                            a = &pods[i];
                            b = &pods[j];
                        }
                    }
                }

                // move all the pods until the date we selected
                for (Pod& p : pods)
                {
                    p.position += dt * p.speed;

                    // collisions with checkpoints (do we need to be as accurate as pod collisions?)
                    if (distSqr(p.position, checkpoints[p.checkpointId]) < checkpointRadiusSqr)
                    {
                        p.checkpointId = (p.checkpointId + 1) % cp_count;
                        ++p.checkpointsPassed;
                    }
                }

                // a and b are colliding -> elastic collision
                if (a != nullptr && b != nullptr)
                {
                    //cerr << "collision at t=" << t << " + dt=" << dt;
                    //cerr << " between "<<a->id<<" and "<<b->id<<endl;
                    Rebound(*a, *b);
                }

                t+=dt;
            }
        }
        void Friction(vector<Pod>& pods) const
        {
            for (Pod& p : pods)
            {
                p.speed *= frictionFactor;
            }
        }
        void EndTurn(vector<Pod>& pods) const
        {
            for (Pod& p : pods)
            {
                p.speed = myVector{ (float)p.speed.x, (float)p.speed.y };
                p.position = myVector{round(p.position.x), round(p.position.y)};
                //print(p);
            }
        }
};
class Solver
{
    private:
        vector<Solution> solutions;
        Simulation* sim;

    public:
        Solver(Simulation* parSimulation)
        {
            sim = parSimulation; // sim != nullptr
            InitPopulation();
            FirstTurnBoostHack();
        }
        const Solution& Solve(const vector<Pod>& pods, int time)
        {
            // we can keep iterating until we spend too much time
            using namespace std::chrono;

            auto start = high_resolution_clock::now();
            auto keepSolving = [&start, time]() -> bool
            {
                auto now = high_resolution_clock::now();
                auto d = duration_cast<milliseconds>(now - start);
                return (d.count() < time);
            };

            // init this turn
            for (int i = 0; i < SOLUTIONCOUNT; ++i)
            {
                // re-use previous solutions (by shifting them by 1 turn)
                Solution& s = solutions[i];
                ShiftByOneTurn(s);
                ComputeScore(s, pods);
            }

            int step = 0;
            while (keepSolving())
            {
                // build and rate mutated versions of our solutions
                for (int i = 0; i < SOLUTIONCOUNT; ++i)
                {
                    Solution& newSol = solutions[SOLUTIONCOUNT + i];
                    newSol = solutions[i];
                    Mutate(newSol);
                    ComputeScore(newSol, pods);
                }

                // sort the solutions by score (the worst ones will be replaced during the next step)
                std::sort( solutions.begin(), solutions.end(),
                           [](const Solution& a, const Solution& b) { return a.score > b.score; }
                         );
                ++step;
            }

            cerr << step << " evolution steps - best score: " << solutions[0].score << endl;
            return solutions[0];
        }

    private:
        void InitPopulation()
        {
            // 0 to (SOLUTIONCOUNT-1): actual solutions from the previous turn
            // SOLUTIONCOUNT to (2*SOLUTIONCOUNT-1): temporary solutions from Solve()
            solutions.resize(2*SOLUTIONCOUNT);

            // make starting solutions random
            for (int s=0; s<SOLUTIONCOUNT; s++)
                for (int t=0; t<SIMULATIONTURNS; t++)
                    for (int i=0; i<2; i++)
                        Randomize(solutions[s][t][i]);
        }
        void FirstTurnBoostHack()
        {
            // hack: just set the boost here if possible, don't waste time later since there's only 1 boost per race anyway...
            const float d = distSqr(sim->Checkpoints()[0], sim->Checkpoints()[1]); // size>1
            constexpr float boostDistanceThreshold = 900000.f;
            if (d < boostDistanceThreshold)
                return;

            cerr << "Boosting the first turn" << endl;
            for (int i=0; i<2; i++)
            {
                for (int s=0; s<SOLUTIONCOUNT; ++s)
                    solutions[s][0][i].boost = true;
            }
        }
        void Randomize(Move& m, bool modifyAll = true) const
        {
            // if !modifyAll, we need to select which value to modify

            // TODO: find a cleaner way to rewrite this (but keep the probabilities compile time)...
            constexpr int all=-1, rotation=0, thrust=1, shield=2, boost=3;
            constexpr int         pr=5      , pt=pr+5,  ps=pt+1,  pb=ps+0; // note: (pb-ps)==0 because we're using FirstTurnBoostHack

            const int i = modifyAll ? -1 : rnd(0, pb);
            const int valueToModify = [i, modifyAll]() -> int
            {
                if (modifyAll)
                    return true;
                if (i<=pr)
                    return rotation;
                if (i<=pt)
                    return thrust;
                if (i<=ps)
                    return shield;
                return boost;
            }();
            auto modifyValue = [valueToModify](int parValue)
            {
                if (valueToModify == all)
                    return true;
                return valueToModify == parValue;
            };

            if (modifyValue(rotation))
            {
                // arbitrarily give more weight to -maxRotation, 0, maxRotation
                const int r = rnd(-2*maxRotation, 3*maxRotation);
                if (r > 2*maxRotation)
                    m.rotation = 0;
                else
                    m.rotation = clamp(r, -maxRotation, maxRotation);
            }
            if (modifyValue(thrust))
            {
                // arbitrarily give more weight to 0, maxThrust
                const int r = rnd(-0.5f * maxThrust, 2*maxThrust);
                m.thrust = clamp(r, 0, maxThrust);
            }
            if (modifyValue(shield))
            {
                if (!modifyAll || (rnd(0,10)>6)) // don't swap it for every init
                    m.shield = !m.shield;
            }
            if (modifyValue(boost))
            {
                if (!modifyAll || (rnd(0,10)>6)) // don't swap it for every init
                    m.boost = !m.boost;
            }
        }
        void ShiftByOneTurn(Solution& s) const
        {
            for (int t=1; t<SIMULATIONTURNS; t++)
                for (int i=0; i<2; i++)
                    s[t-1][i] = s[t][i];

            // create a new random turn at the end
            for (int i=0; i<2; i++)
            {
                Move& m = s[SIMULATIONTURNS-1][i];
                Randomize(m);
            }
        }
        void Mutate(Solution& s) const
        {
            // mutate one value for a random t,i
            int k = rnd(0, 2*SIMULATIONTURNS);
            Move& m = s[k/2][k%2];

            int h = rnd(0,1);
            Randomize(m, false);
        }
        int ComputeScore(Solution& sol, const vector<Pod>& pods) const
        {
            vector<Pod> podsCopy = pods;
            sim->PlaySolution(podsCopy, sol);
            sol.score = RateSolution(podsCopy);
            return sol.score;
        }
        int RateSolution(vector<Pod>& pods) const
        {
            auto podScore = [&](const Pod& p) -> int
            {
                // the max distance between 2 checkpoints in the map (of size 16k*9k) is about 18k
                // we rate each checkpoint passed higher than that:
                constexpr int cpFactor = 30000;
                const int distToCp = dist(p.position, sim->Checkpoints()[p.checkpointId]);
                return cpFactor*p.checkpointsPassed - distToCp;
            };
            for (Pod& p : pods)
                p.score = podScore(p);

            const int myRacerIndex = (pods[0].score > pods[1].score) ? 0 : 1;
            const Pod& myRacer = pods[myRacerIndex];
            const Pod& myInterceptor = pods[1-myRacerIndex];

            const Pod& opponentRacer = (pods[2].score > pods[3].score) ? pods[2] : pods[3];

            if (myRacer.checkpointsPassed > sim->MaxCheckpoints())
                return INFINITY; // I won

            if (opponentRacer.checkpointsPassed > sim->MaxCheckpoints())
                return -INFINITY; // opponent won

            // how far ahead are we?
            const int aheadScore = myRacer.score - opponentRacer.score;

            // can my interceptor block the opponent racer?
            const myVector opponentCheckpoint = sim->Checkpoints()[opponentRacer.checkpointId];
            const int interceptorScore = -dist(myInterceptor.position, opponentCheckpoint);

            constexpr int aheadBias = 2; // being ahead is better than blocking the opponent
            return aheadScore*aheadBias + interceptorScore;
        }
};

// class dependent function prototype
void PrintSolution(const Solution& s);
void ConvertSolutionToOutput(const Solution& sol, vector<Pod>& pods);
void UpdatePodsShieldBoostForNextTurn(const Solution& sol, vector<Pod>& pods);

int main()
{
    constexpr float timeoutSafeguard = .95f;
    int availableTime = 0;
    int step = 0;

    Simulation simulation;
    myVector firstCheckpoint = simulation.InitCheckpointsFromInput();;

    Solver solver { &simulation };
    vector<Pod> pods(4);

    while (1)
    {
        for (int pod_idx = 0; pod_idx < 4; pod_idx++)
        {
            UpdatePodFromInput(pods[pod_idx], pod_idx);

            if (step == 0)
            {
                OverrideAngle(pods[pod_idx], firstCheckpoint);
            }
        }

        if (step == 0) availableTime = timeoutFirstTurn;
        else           availableTime = timeout;

        const Solution& s = solver.Solve(pods, availableTime * timeoutSafeguard);
        ConvertSolutionToOutput(s, pods);
        UpdatePodsShieldBoostForNextTurn(s, pods);
        
        step++;
    }
}

//Func declaration
bool operator==(const myVector& a, const myVector& b)
{
    return a.x == b.x && a.y == b.y;
}
myVector operator+(const myVector& a, const myVector& b)
{
    return myVector(a.x+b.x, a.y+b.y);
}
myVector operator+=(myVector& a, const myVector& b)
{
    a = a+b;
    return a;//a = a+b;
}
myVector operator-(const myVector& a, const myVector& b)
{
    return myVector(a.x-b.x, a.y-b.y);
}
myVector operator*(float k, const myVector& a)
{
    return myVector(k*a.x, k*a.y);
}
myVector operator*=(myVector& a, float k)
{
    a = k*a;
    return a;
}
ostream& operator<<(ostream& os, const myVector& v)
{
    os << v.x << " " << v.y;
    return os;
}
float distSqr(const myVector& a, const myVector& b)
{
    return (a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y);
}
float dist(const myVector& a, const myVector& b)
{
    return sqrt(distSqr(a, b));
}
float dot(const myVector& a, const myVector& b)
{
    return a.x*b.x + a.y*b.y;
}
void ManageShield(bool turnOn, Pod& p)
{
  if (turnOn)
    p.shieldCd = shieldCooldown;
  else if (p.shieldCd > 0)
    --p.shieldCd;
}
int Mass(const Pod& p)
{
    if (p.shieldCd == shieldCooldown)
        return 10;
    return 1;
}
float CollisionTime(Pod& p1, Pod& p2)
{
    const myVector dP = (p2.position - p1.position);
    const myVector dS = (p2.speed - p1.speed);

    constexpr float eps = 0.000001f; // float precision...

    // we're looking for t such that:
    // |            p2(t)           -           p1(t)           | < 2*podRadius
    // |(p2.position + t*p2.speed)  - (p1.position + t*p1.speed)| < 2*podRadius
    // |(p2.position - p1.position) -  t*(p2.speed - p1.speed)  | < 2*podRadius
    // |         dP                 +            t*dS           | < 2*podRadius
    // t^2 dS^2 + t 2dPdS + dP^2 < 4 podRadius^2
    // t^2  a   + t   b   +   c      = 0;

    const float a = dot(dS, dS);
    if (a < eps) // moving away from each other
        return INFINITY;

    const float b = -2.f*dot(dP, dS);
    const float c = dot(dP,dP) - 4.f*podRadiusSqr;

    const float delta = b*b - 4.f*a*c;
    if (delta < 0.f) // no solution
        return INFINITY;

    const float t = (b - sqrt(delta)) / (2.f * a);
    if (t <= eps)
        return INFINITY;

    return t;
}
void Rebound(Pod& a, Pod& b)
{
    // https://en.wikipedia.org/wiki/Elastic_collision#Two-dimensional_collision_with_two_moving_objects
    const float mA = Mass(a);
    const float mB = Mass(b);

    const myVector dP = b.position - a.position;
    const float AB = dist(a.position, b.position);

    const myVector u = (1.f / AB) * dP; // rebound direction

    const myVector dS = b.speed - a.speed;

    const float m = (mA * mB) / (mA + mB);
    const float k = dot(dS, u);

    const float impulse = -2.f * m * k;
    const float impulseToUse = clamp(impulse, -minImpulse, minImpulse);

    a.speed += (-1.f/mA) * impulseToUse * u;
    b.speed += (1.f/mB) * impulseToUse * u;
}
void PrintPod(const Pod& p)
{
    //if (p.id != 1) return;

    cerr << "position: " << p.position << endl;
    cerr << "speed:    " << p.speed << endl;
    cerr << "angle:    " << p.angle << endl;
    cerr << "nextCpId: " << p.checkpointId;
    cerr << "  passed: " << p.checkpointsPassed << endl;
    cerr << "canBoost: " << p.canBoost;
    cerr << "  shieldCd: " << p.shieldCd << endl << endl;
}
void PrintSolution(const Solution& s)
{
    for (int t=0; t<SIMULATIONTURNS; t++)
    {
        for (int i=0; i<2; i++)
        {
            const Move& m = s[t][i];
            cerr << m.rotation << "     " << m.thrust << endl;
        }
        cerr << endl;
    }
}
void UpdatePodFromInput(Pod& p, int id)
{
    int x, y, vx, vy, angle, nextCheckPointId;
    cin >> x >> y >> vx >> vy >> angle >> nextCheckPointId; cin.ignore();

    //p.id = id;
    p.position = myVector(x,y);
    p.speed = myVector(vx, vy);
    p.angle = angle;
    if (p.checkpointId != nextCheckPointId)
        ++p.checkpointsPassed;
    p.checkpointId = nextCheckPointId;

    PrintPod(p);
}
void ConvertSolutionToOutput(const Solution& sol, vector<Pod>& pods)
{
    constexpr int t = 0; // we must output the first turn of the solution

    for (int i=0; i<2; i++)
    {
        const Pod& p = pods[i];
        const Move& m = sol[t][i];

        // generate coordinates from the rotation
        const float angle = (p.angle + m.rotation) % 360;
        const float angleRad = angle * PI / 180.f;

        constexpr float targetDistance = 10000.f; // compute the target arbitrarily far enough, to avoid rounding errors
        const myVector direction{ targetDistance*cos(angleRad),targetDistance*sin(angleRad) };
        const myVector target = p.position + direction;

        cout << round(target.x) << " " << round(target.y) << " ";

        if (m.shield)
            cout << "SHIELD";
        else if (m.boost)
            cout << "BOOST";
        else
            cout << m.thrust;

        cout << endl;
    }
}
void UpdatePodsShieldBoostForNextTurn(const Solution& sol, vector<Pod>& pods)
{
    // those values aren't read from the input, so we have to compute them
    for (int i=0; i<2; i++)
    {
        Pod& p = pods[i];
        const Move& m = sol[0][i];

        ManageShield(m.shield, p);

        if (p.shieldCd == 0 && m.boost)
            p.canBoost = false;
    }
}
void OverrideAngle(Pod& p, const myVector& target)
{
    // make p face target immediately, ignoring turn rate (maxRotation): this is necessary to init the first turn
    const myVector dir = (target - p.position).normalized();
    float a = acos(dir.x) * 180.f / PI;
    if (dir.y < 0)
        a = (360.f - a);
    p.angle = a;
}