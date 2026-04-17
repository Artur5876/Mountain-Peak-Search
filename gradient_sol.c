#include "gradient.h"
#include <math.h>

static void get_view(int y, int x, float view[VIEW_SIZE][VIEW_SIZE]) {
    generate_view(view, y, x);
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
path_point find_highest_point(void) {
    int x = 0, y = 0;
    float view[VIEW_SIZE][VIEW_SIZE];
    int max_iterations = 1000;   // safeguard
    int iter = 0;
    const int STEP = VIEW_SIZE /2;


    while (iter < max_iterations) {
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

        if (mag > 0.1f) {
            //step size between 1 and VIEW_SIZE(proportional to magnitude)
            float step = mag / 10.0f;
            if (step < 1.0f) step = 1.0f;
            if (step > VIEW_SIZE) step = VIEW_SIZE;
            int dx = (int)(step * gx / mag);
            int dy = (int)(step * gy / mag);
            x += dx;
            y += dy;
        } else {
            //move to the highest cell in the view if gradient is tiny
            int max_dy, max_dx;
            view_max_pos(view, &max_dy, &max_dx);
            x += max_dx - VIEW_SIZE/2;
            y += max_dy - VIEW_SIZE/2;
        }

        //keep coordinates non-negative;
        if (x < 0) x=0;
        if(y < 0) y =0;
    }
}
