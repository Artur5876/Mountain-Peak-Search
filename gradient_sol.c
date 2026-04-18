#include "gradient.h"
#include <math.h>

//calls counter
static int call_count =0;
#define MAX_CALLS 900

static void get_view(int y, int x, float view[VIEW_SIZE][VIEW_SIZE]) {
    if (call_count >= MAX_CALLS) return;
    generate_view(view, y, x);
    ++call_count;
}

static float view_max_pos(const float view[VIEW_SIZE][VIEW_SIZE],
                          int *max_dy, int *max_dx) {
    float max_val = -1e9f;
    *max_dy = *max_dx = 0;
    for (int i = 0; i < VIEW_SIZE; i++) {
        for (int j = 0; j < VIEW_SIZE; j++) {
            if (view[i][j] > max_val) {
                max_val = view[i][j];
                *max_dy = i;
                *max_dx = j;
            }
        }
    }
    return max_val;
}

static int is_local_peak(const float view[VIEW_SIZE][VIEW_SIZE]) {
    float center = view[VIEW_SIZE/2][VIEW_SIZE/2];
    if (center < 0.0f) return 0;
    for (int i = 0; i < VIEW_SIZE; i++) {
        for (int j = 0; j < VIEW_SIZE; j++) {
            if (i == VIEW_SIZE/2 && j == VIEW_SIZE/2) continue;
            float v = view[i][j];
            if (v >= 0.0f && v >= center - 0.01f) return 0;
        }
    }
    return 1;
}

static void compute_gradient(const float view[VIEW_SIZE][VIEW_SIZE], 
                            float *gx, float *gy) {
    int half = VIEW_SIZE / 2;
    float left = 0, right = 0, up = 0,down = 0;
    int lc = 0, rc = 0, uc = 0, dc = 0;
    for (int i =0; i < VIEW_SIZE; ++i) {
        for (int j =0; j < VIEW_SIZE; j++) {
            float v = view[i][j];
            if (v < 0.0f) continue;

            if (j > half) {right += v; ++lc;}
            else if (j < half) {left += v; ++rc;}

            if (i < half) {up += v; ++uc;}
            else if (i > half) {down += v; ++dc;}
        }
    }
    *gx = (rc ? right/rc : 0) - (lc ? left/lc : 0);
    *gy = (dc ? down/dc : 0) - (up ? up/uc : 0);
}

static void escape_plateau(int *x, int *y, float current_alt){
    int layer = 1;

    while(call_count < MAX_CALLS - 50) {
        //iterate all over the square perimeter
        for (int dy = -layer; dy <= layer; ++dy) {
            for (int dx = -layer; dx <= layer; ++dx) {
                //consider cells at the border only
                if (abs(dy) != layer && abs(dx) != layer) continue;

                //new possible coordinates calculation
                int nx = *x + dx * VIEW_SIZE/2;
                int ny = *y + dy * VIEW_SIZE/2;

                //view centered at coordinates radious
                float view[VIEW_SIZE][VIEW_SIZE];
                get_view(ny, nx, view);

                //find the max altitude
                int max_dy, max_dx;
                float max_val = view_max_pos(view, &max_dy, &max_dx);

                //if we found a slope;
                if (fabs(max_val - current_alt) > 0.1f) {
                    *x = nx;
                    *y = ny;
                    return;
                }
            }
        }
        ++layer; //expanssion of the next layer;
    } 
}

path_point find_highest_point(void) {
    int x = 0, y = 0;
    float view[VIEW_SIZE][VIEW_SIZE];
    const int STEP = VIEW_SIZE / 2;      // base step size

    //momentum variables
    float vx = 0.0f, vy = 0.0f;
    int flat_steps = 0;   //counter for consecutive flat steps

    int max_iterations = 1000;
    int iter = 0;

    while (call_count < MAX_CALLS - 5) { //leave room for peak declaration
        iter++;
        get_view(y, x, view);

        if (is_local_peak(view)) {
            if (declare_peak(x, y)) {
                path_point p = {x, y};
                return p;
            }
        }

        float gx, gy;
        compute_gradient(view, &gx, &gy);
        float mag = sqrt(gx*gx + gy*gy);

        // PLATEAU DETECTION
        if (mag < 0.1f) {        //if gradient is nearly zero -->possible plateau
            flat_steps++;
            if (flat_steps >= 2) {
                float center = view[VIEW_SIZE/2][VIEW_SIZE/2];
                escape_plateau(&x, &y, center);
                flat_steps =0;
                vx = vy = 0.0f; //reset momnentum
                continue;
            }
        } else {
            flat_steps = 0;      //there is a slope, reset plateau counter
        }
        
        //MOMENTUM‑BASED MOVEMENT
        vx = 0.8f * vx + 0.2f * gx;
        vy = 0.8f * vy + 0.2f * gy;

        float vm = sqrt(vx*vx + vy*vy);
        if (vm > 0.1f) {
            // Step size proportional to velocity magnitude
            float step_scale = vm / 10.0f;
            if (step_scale < 1.0f) step_scale = 1.0f;
            if (step_scale > VIEW_SIZE) step_scale = VIEW_SIZE;
            int dx = (int)(step_scale * vx / vm);
            int dy = (int)(step_scale * vy / vm);
            x += dx;
            y += dy;
        } else {
            // If velocity is tiny, move toward the maximum cell in the view
            int max_dy, max_dx;
            view_max_pos(view, &max_dy, &max_dx);
            x += max_dx - VIEW_SIZE/2;
            y += max_dy - VIEW_SIZE/2;
        }

        //keep coordinates non‑negative
        if (x < 0) x = 0;
        if (y < 0) y = 0;
    }

    //if we exhaust iterations (should not happen), return best guess
    path_point fallback = {x, y};
    return fallback;
}
