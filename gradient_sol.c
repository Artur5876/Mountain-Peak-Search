#include "gradient.h"

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

path_point find_highest_point(void) {
    int x = 0, y = 0;
    float view[VIEW_SIZE][VIEW_SIZE];
    int max_iterations = 1000;   // safeguard
    int iter = 0;

    while (iter < max_iterations) {
        iter++;
        get_view(y, x, view);
        if (is_local_peak(view)) {
            if (declare_peak(x, y)) {
                path_point p = {x, y};
                return p;
            }
        }
        int max_dy, max_dx;
        view_max_pos(view, &max_dy, &max_dx);
        int new_x = x + max_dx - VIEW_SIZE/2;
        int new_y = y + max_dy - VIEW_SIZE/2;

        //if there is no progress, then break
        if (new_x == x && new_y == y) {
            break;
        }
        x = new_x;
        y = new_y;
    }
    //declare whatever we have (might be wrong)
    declare_peak(x, y);
    path_point p = {x, y};
    return p;
}
