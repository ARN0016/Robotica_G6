#ifndef SIMPLE_DWA_H
#define SIMPLE_DWA_H

#include <Eigen/Dense>
#include <vector>

// Simple Dynamic Window Approach that follows a person and avoids obstacles (see dwa.pdf and handout.pdf).
// Robot frame, millimetres: x right, y forward. Heading h is measured counter-clockwise from +y, so the forward
// direction is (-sin h, cos h). w > 0 turns counter-clockwise (left), as setSpeedBase(0, adv, rot) expects.
class SimpleDWA
{
    public:
        struct Command
        {
            float adv = 0.f;    // mm/s, forward, never negative
            float rot = 0.f;    // rad/s, + = counter-clockwise
        };

        struct Arc                                  // one candidate, kept for drawing
        {
            float v = 0.f;                          // mm/s
            float w = 0.f;                          // rad/s
            bool valid = false;                     // false if it collides or cannot brake in time
            std::vector<Eigen::Vector2f> path;      // mm, simulated positions, one per step
        };

        // TODO 1. lidar: points (x, y, z) in mm in the robot frame; person: tracked person (x, y) in mm.
        // Returns the (x, y) of the points that are obstacles.
        std::vector<Eigen::Vector2f> obstacle_points(const std::vector<Eigen::Vector3f>& lidar,
                                                     const Eigen::Vector2f& person) const;

        // TODO 4. One DWA cycle: the (adv, rot) to send to the base. Call it once per compute().
        Command update(const Eigen::Vector2f& person, const std::vector<Eigen::Vector2f>& points);

        // Optional, task 6 (PID <-> DWA switch): clearance of the arc that cmd would drive, d_min - R in mm,
        // -infinity if it collides. Reuses rollout().
        float clearance(Command cmd, const std::vector<Eigen::Vector2f>& points) const;

        void reset();                               // forgets prev_w_ (call it when the robot stops)
        void set_prev_w(float w) { prev_w_ = w; }   // task 6: when the PID drives, keep the window where the robot is

        const std::vector<Arc>& arcs() const { return arcs_; }   // last cycle's candidates
        int chosen() const { return chosen_; }                   // index into arcs(), -1 if the robot stopped

    private:
        struct Rollout
        {
            bool hit = false;                       // true if the arc collides
            Eigen::Vector2f end = Eigen::Vector2f::Zero();   // mm, position at the end of the arc
            float heading = 0.f;                    // rad, heading at the end of the arc
            float d_min = 0.f;                      // mm, closest distance to any point along the arc
        };

        // TODO 2. Simulates constant (v, w) for HORIZON seconds in steps of STEP; fills path if it is not null.
        Rollout rollout(float v, float w, const std::vector<Eigen::Vector2f>& points,
                        std::vector<Eigen::Vector2f>* path) const;

        // TODO 3. Cost of a valid arc, lower is better.
        float cost(float v, const Rollout& r, const Eigen::Vector2f& person) const;

        // obstacle points
        static constexpr float Z_MIN = 130.f;       // mm, below this is the floor
        static constexpr float Z_MAX = 1500.f;      // mm, above this the robot passes under
        static constexpr float RANGE_MAX = 3000.f;  // mm, farther points are ignored
        static constexpr float SELF_RANGE = 400.f;  // mm, closer points are the robot itself
        static constexpr float PERSON_RADIUS = 500.f;   // mm, points this close to the person are the person

        // sampling and rollout
        static constexpr float R = 400.f;           // mm, robot radius 365 + margin
        static constexpr float HORIZON = 2.f;       // s
        static constexpr float STEP = 0.1f;         // s
        static constexpr int N_V = 11;              // v = 0, 100, ..., V_MAX
        static constexpr int N_W = 21;              // w from prev_w_ - W_WINDOW to prev_w_ + W_WINDOW
        static constexpr float V_MAX = 1000.f;      // mm/s
        static constexpr float W_MAX = 1.5f;        // rad/s
        static constexpr float W_WINDOW = 1.f;      // rad/s
        static constexpr float A_BRAKE = 500.f;     // mm/s^2, the real base brakes a wheel at 550

        // cost
        static constexpr float D_REF = 1000.f;      // mm, desired distance to the person
        static constexpr float DIST_SCALE = 2000.f; // mm, distance error that costs 1
        static constexpr float CLEAR_SAFE = 500.f;  // mm, clearance above this costs nothing
        static constexpr float W_HEAD = 1.f;
        static constexpr float W_DIST = 1.f;
        static constexpr float W_CLEAR = 3.f;
        static constexpr float W_VEL = 1.f;

        // progress rule
        static constexpr float FAR = 1100.f;        // mm, the person is far beyond this
        static constexpr float V_PROGRESS = 150.f;  // mm/s, while far, slower arcs only if nothing faster is valid

        float prev_w_ = 0.f;                        // rad/s, w chosen last cycle
        std::vector<Arc> arcs_;
        int chosen_ = -1;
};

#endif // SIMPLE_DWA_H
