#include "simple_dwa.h"

#include <iostream>
#include <algorithm>
#include <cmath>
#include <limits>


void SimpleDWA::reset()
{
    prev_w_ = 0.f;
    arcs_.clear();
    chosen_ = -1;
}

std::vector<Eigen::Vector2f> SimpleDWA::obstacle_points(const std::vector<Eigen::Vector3f>& lidar,
                                                        const Eigen::Vector2f& person) const
{
    std::vector<Eigen::Vector2f> out;
    // TODO 1: keep (x, y) of the points that
    //   - have Z_MIN < z < Z_MAX
    //   - are closer than RANGE_MAX and farther than SELF_RANGE from the robot (origin)
    //   - are farther than PERSON_RADIUS from the person
    // Enfoque: descartar cada punto con "continue" y comparar distancias al cuadrado (sin raices).
    for (const auto& p : lidar)
    {
        if (p.z() <= Z_MIN or p.z() >= Z_MAX)
            continue;                                   // suelo o por encima del robot

        const float robot_d2 = p.x() * p.x() + p.y() * p.y();
        if (robot_d2 >= RANGE_MAX * RANGE_MAX or robot_d2 <= SELF_RANGE * SELF_RANGE)
            continue;                                   // demasiado lejos o es el propio robot

        const float dx = p.x() - person.x();
        const float dy = p.y() - person.y();
        if (dx * dx + dy * dy <= PERSON_RADIUS * PERSON_RADIUS)
            continue;                                   // es la persona

        out.push_back(Eigen::Vector2f(p.x(), p.y()));
    }
    return out;
}

SimpleDWA::Rollout SimpleDWA::rollout(float v, float w, const std::vector<Eigen::Vector2f>& points,
                                      std::vector<Eigen::Vector2f>* path) const
{
    Rollout r;
    r.d_min = std::numeric_limits<float>::infinity();
    // TODO 2: start at x = y = h = 0 and repeat HORIZON / STEP times:
    //   - move: x -= v sin(h) STEP;  y += v cos(h) STEP;  h += w STEP
    //   - if path is not null, append (x, y) to *path
    //   - for every point q: d = distance from (x, y) to q
    //       if d < R and d < |q|  ->  r.hit = true, return r   (|q| = distance from the robot now)
    //       r.d_min = min(r.d_min, d)
    //   at the end fill r.end and r.heading
    // Enfoque: se trabaja con distancias al cuadrado y solo se hace una raiz al final.
    float x = 0.f;
    float y = 0.f;
    float h = 0.f;
    float d2_min = std::numeric_limits<float>::infinity();

    for (int i = 0; i < 20; ++i)                        // HORIZON / STEP = 2 / 0.1 = 20 pasos
    {
        x -= v * std::sin(h) * STEP;
        y += v * std::cos(h) * STEP;
        h += w * STEP;

        if (path != nullptr)
            path->push_back(Eigen::Vector2f(x, y));

        for (const auto& q : points)
        {
            const float dx = q.x() - x;
            const float dy = q.y() - y;
            const float d2 = dx * dx + dy * dy;
            if (d2 < R * R and d2 < q.x() * q.x() + q.y() * q.y())
            {
                r.hit = true;
                return r;
            }
            if (d2 < d2_min)
                d2_min = d2;
        }
    }
    r.end = Eigen::Vector2f(x, y);
    r.heading = h;
    r.d_min = std::sqrt(d2_min);
    return r;
}

float SimpleDWA::cost(float v, const Rollout& r, const Eigen::Vector2f& person) const
{
    // TODO 3: g = person - r.end
    //   heading term: h* = atan2(-g.x, g.y); c_h = |wrap(h* - r.heading)| / pi   (wrap to [-pi, pi])
    //   distance term: c_d = | |g| - D_REF | / DIST_SCALE
    //   clearance term: c_c = max(0, 1 - (r.d_min - R) / CLEAR_SAFE)^2
    //   speed term: c_v = 1 - v / V_MAX
    //   return W_HEAD c_h + W_DIST c_d + W_CLEAR c_c + W_VEL c_v
    // Enfoque: sin atan2. El rumbo se mide con el coseno del angulo entre hacia donde mira el
    //          robot y hacia donde esta la persona. Ademas, el coste empuja al robot a acercarse.
    const float gx = person.x() - r.end.x();
    const float gy = person.y() - r.end.y();
    const float dist = std::sqrt(gx * gx + gy * gy);

    // rumbo: 0 si el robot mira a la persona, 0.5 si esta de lado, 1 si esta detras
    float c_h = 0.f;
    if (dist > 1.f)                                     // evita dividir por casi cero
    {
        const float fx = -std::sin(r.heading);          // hacia donde mira el robot
        const float fy = std::cos(r.heading);
        const float cos_angle = (fx * gx + fy * gy) / dist;
        c_h = (1.f - cos_angle) / 2.f;
    }

    // distancia: quedarse lejos cuesta el doble que pasarse de cerca, para que el robot se acerque
    float c_d = 0.f;
    if (dist > D_REF)
        c_d = 2.f * (dist - D_REF) / DIST_SCALE;
    else
        c_d = (D_REF - dist) / DIST_SCALE;

    // holgura con los obstaculos
    float c_c = 1.f - (r.d_min - R) / CLEAR_SAFE;
    if (c_c < 0.f)
        c_c = 0.f;
    c_c = c_c * c_c;

    // velocidad: solo interesa ir rapido si todavia falta para llegar a la persona
    float c_v = 0.f;
    if (dist > D_REF)
        c_v = 1.f - v / V_MAX;

    return W_HEAD * c_h + W_DIST * c_d + W_CLEAR * c_c + W_VEL * c_v;
}

SimpleDWA::Command SimpleDWA::update(const Eigen::Vector2f& person, const std::vector<Eigen::Vector2f>& points)
{
    arcs_.clear();
    chosen_ = -1;
    // TODO 4: for N_V values of v in [0, V_MAX] and N_W values of w in [prev_w_ - W_WINDOW, prev_w_ + W_WINDOW]
    //         (clipped to [-W_MAX, W_MAX]):
    //   - r = rollout(v, w, points, &arc.path)
    //   - valid = not r.hit and v <= sqrt(2 A_BRAKE max(r.d_min - R, 0))
    //   - push the Arc into arcs_; if valid, compute its cost
    //   - remember the cheapest valid arc, and separately the cheapest one with v >= V_PROGRESS
    // TODO 5: progress rule: if |person| > FAR and a fast arc exists, choose the cheapest fast one,
    //         otherwise the cheapest valid one. None valid -> prev_w_ = 0, return {0, 0}.
    //         Otherwise set chosen_ and prev_w_ and return its (v, w).
    // Enfoque: primero se generan todos los arcos y su coste (infinito si no es valido);
    //          despues se recorre la lista para elegir el mejor.
    std::vector<float> costs;
    const float inf = std::numeric_limits<float>::infinity();

    // ventana de w recortada a [-W_MAX, W_MAX]
    float w_lo = prev_w_ - W_WINDOW;
    float w_hi = prev_w_ + W_WINDOW;
    if (w_lo < -W_MAX)
        w_lo = -W_MAX;
    if (w_hi > W_MAX)
        w_hi = W_MAX;
    const float w_step = (w_hi - w_lo) / (N_W - 1);

    // 1) generar arcos
    for (int i = 0; i < N_V; ++i)
    {
        for (int j = 0; j < N_W; ++j)
        {
            Arc arc;
            arc.v = V_MAX * i / (N_V - 1);
            arc.w = w_lo + w_step * j;

            const Rollout r = rollout(arc.v, arc.w, points, &arc.path);

            float c = inf;
            if (not r.hit)
            {
                float free_dist = r.d_min - R;          // espacio libre para frenar
                if (free_dist < 0)
                    free_dist = 0;
                if (arc.v * arc.v <= 2 * A_BRAKE * free_dist)   // equivale a v <= sqrt(2 A d)
                {
                    arc.valid = true;
                    c = cost(arc.v, r, person);
                }
            }
            arcs_.push_back(arc);
            costs.push_back(c);
        }
    }

    // 2) elegir: si la persona esta lejos, primero se miran solo los arcos rapidos
    const bool far = person.x() * person.x() + person.y() * person.y() > FAR * FAR;

    if (far)
    {
        float best_cost = inf;
        for (int k = 0; k < static_cast<int>(arcs_.size()); ++k)
            if (arcs_[k].valid and arcs_[k].v >= V_PROGRESS and costs[k] < best_cost)
            {
                best_cost = costs[k];
                chosen_ = k;
            }
    }
    if (chosen_ < 0)                                    // no hay arco rapido (o la persona esta cerca)
    {
        float best_cost = inf;
        for (int k = 0; k < static_cast<int>(arcs_.size()); ++k)
            if (arcs_[k].valid and costs[k] < best_cost)
            {
                best_cost = costs[k];
                chosen_ = k;
            }
    }

    if (chosen_ < 0)                                    // ninguno valido: parar
    {
        prev_w_ = 0.f;
        return {0.f, 0.f};
    }
    prev_w_ = arcs_[chosen_].w;
    return {arcs_[chosen_].v, arcs_[chosen_].w};
}

float SimpleDWA::clearance(Command cmd, const std::vector<Eigen::Vector2f>& points) const
{
    // TODO 6 (optional): rollout of (cmd.adv, cmd.rot); -infinity if it hits, else d_min - R
    const Rollout r = rollout(cmd.adv, cmd.rot, points, nullptr);
    if (r.hit)
        return -std::numeric_limits<float>::infinity();
    return r.d_min - R;
}