#include "gradient.h"
#include <math.h>

static int call_count = 0;
#define MAX_CALLS     900
#define LANDSCAPE_MAX 200
#define HALF_VIEW     (VIEW_SIZE / 2)

#define GSS 0.61803398875f

static void get_view(int y, int x, float view[VIEW_SIZE][VIEW_SIZE]) {
    if (call_count >= MAX_CALLS) return;
    generate_view(view, y, x);
    ++call_count;
}

static float view_max_pos(const float view[VIEW_SIZE][VIEW_SIZE],
                          int *out_dy, int *out_dx) {
    float best = -1e9f;
    *out_dy = *out_dx = 0;
    for (int i = 0; i < VIEW_SIZE; i++)
        for (int j = 0; j < VIEW_SIZE; j++)
            if (view[i][j] >= 0.0f && view[i][j] > best) {
                best   = view[i][j];
                *out_dy = i - HALF_VIEW;   // offset from centre
                *out_dx = j - HALF_VIEW;
            }
    return best;
}

static int is_local_peak(const float view[VIEW_SIZE][VIEW_SIZE]) {
    float c = view[HALF_VIEW][HALF_VIEW];
    if (c < 0.0f) return 0;
    for (int i = 0; i < VIEW_SIZE; i++)
        for (int j = 0; j < VIEW_SIZE; j++) {
            if (i == HALF_VIEW && j == HALF_VIEW) continue;
            float v = view[i][j];
            if (v >= 0.0f && v >= c - 0.01f) return 0;   // tolerance for fp noise
        }
    return 1;
}

// evaluate view at (y,x) and update global best if better
static float sample(int y, int x,
                    int *best_y, int *best_x, float *best_val) {
    float view[VIEW_SIZE][VIEW_SIZE];
    int dy, dx;
    get_view(y, x, view);
    float val = view_max_pos(view, &dy, &dx);
    if (val > *best_val) {
        *best_val = val;
        *best_y   = y + dy;
        *best_x   = x + dx;
    }
    return val;
}

static int clamp(int v) {
    return v < 0 ? 0 : (v > LANDSCAPE_MAX ? LANDSCAPE_MAX : v);
}

// golden section search along horizontal line y
static int gss_x(int y, int lo, int hi,
                 int *best_y, int *best_x, float *best_val) {
    int x1 = hi - (int)(GSS * (hi - lo));
    int x2 = lo + (int)(GSS * (hi - lo));
    float f1 = sample(y, x1, best_y, best_x, best_val);
    float f2 = sample(y, x2, best_y, best_x, best_val);

    while (hi - lo > VIEW_SIZE && call_count < MAX_CALLS - 20) {
        if (f1 < f2) {
            lo = x1; x1 = x2; f1 = f2;
            x2 = lo + (int)(GSS * (hi - lo));
            f2 = sample(y, x2, best_y, best_x, best_val);
        } else {
            hi = x2; x2 = x1; f2 = f1;
            x1 = hi - (int)(GSS * (hi - lo));
            f1 = sample(y, x1, best_y, best_x, best_val);
        }
    }
    return (lo + hi) / 2;
}

// golden section search along vertical line x
static int gss_y(int x, int lo, int hi,
                 int *best_y, int *best_x, float *best_val) {
    int y1 = hi - (int)(GSS * (hi - lo));
    int y2 = lo + (int)(GSS * (hi - lo));
    float f1 = sample(y1, x, best_y, best_x, best_val);
    float f2 = sample(y2, x, best_y, best_x, best_val);

    while (hi - lo > VIEW_SIZE && call_count < MAX_CALLS - 20) {
        if (f1 < f2) {
            lo = y1; y1 = y2; f1 = f2;
            y2 = lo + (int)(GSS * (hi - lo));
            f2 = sample(y2, x, best_y, best_x, best_val);
        } else {
            hi = y2; y2 = y1; f2 = f1;
            y1 = hi - (int)(GSS * (hi - lo));
            f1 = sample(y1, x, best_y, best_x, best_val);
        }
    }
    return (lo + hi) / 2;
}

path_point find_highest_point(void) {
    call_count = 0;

    float best_val = -1e9f;
    int   best_y = 0, best_x = 0; //highest point coordinates

    int mid = LANDSCAPE_MAX / 2;

    //  coarse x location along horizontal mid‑line
    int cx = gss_x(mid, 0, LANDSCAPE_MAX,
                   &best_y, &best_x, &best_val);

    // coarse y location along vertical at cx
    int cy = gss_y(cx, 0, LANDSCAPE_MAX,
                   &best_y, &best_x, &best_val);

    // refine x in a narrow window around cx
    int lo = clamp(cx - VIEW_SIZE * 2);
    int hi = clamp(cx + VIEW_SIZE * 2);
    cx = gss_x(cy, lo, hi, &best_y, &best_x, &best_val);

    float view[VIEW_SIZE][VIEW_SIZE];
    int x = clamp(best_x), y = clamp(best_y);

    // 4. hill climb from the best point found so far
    while (call_count < MAX_CALLS - 2) {
        get_view(y, x, view);

        if (is_local_peak(view)) {
            if (declare_peak(x, y)) {
                path_point p = {x, y};
                return p;
            }
            // false positive – move to highest in view
            int dy, dx;
            view_max_pos(view, &dy, &dx);
            x = clamp(x + dx);
            y = clamp(y + dy);
            continue;
        }

        int dy, dx;
        view_max_pos(view, &dy, &dx);
        int nx = clamp(x + dx);
        int ny = clamp(y + dy);

        if (nx == x && ny == y) {   // stuck – assume peak
            declare_peak(x, y);
            path_point p = {x, y};
            return p;
        }
        x = nx;
        y = ny;
    }

    path_point fallback = {x, y};
    return fallback;
}
