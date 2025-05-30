#include <gtk/gtk.h>
#include <cairo.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <limits.h>
#define M_PI 3.14159265358979323846
#define MAX_NODES 50
#define MAX_PACKAGES 100
#define INF 999999
#define CANVAS_WIDTH 1000
#define CANVAS_HEIGHT 1000
#define VEHICLE_SIZE 20
#define ANIMATION_INTERVAL 50

typedef struct {
    int id;
    char name[100];
    double lat, lon;
    double x, y;
    int is_depot;
    int package_count;
} DeliveryPoint;

typedef struct {
    int id;
    int weight;
    int value;
    int priority;
    double carbon_footprint;
    int destination_id;
} Package;

typedef struct {
    int from, to;
    double distance;
    double carbon_emission_factor;
} Route;

typedef struct {
    GtkWidget *window;
    GtkWidget *drawing_area;
    GtkWidget *info_label;
    GtkWidget *route_button;
    GtkWidget *optimize_button;
    GtkWidget *start_button;
    GtkWidget *speed_scale;
    GtkWidget *table_label;
    DeliveryPoint points[MAX_NODES];
    Package packages[MAX_PACKAGES];
    Route routes[MAX_NODES * MAX_NODES];
    int num_points;
    int num_packages;
    int num_routes;
    int *optimal_route;
    int route_length;
    double total_distance;
    double total_emissions;
    int selected_point;
    gboolean show_route;
    gboolean animation_running;
    int current_route_segment;
    double vehicle_progress;
    guint animation_timeout_id;
    double animation_speed;
} DeliveryApp;

DeliveryApp *app;

void dijkstra(int graph[MAX_NODES][MAX_NODES], int src, int dist[], int parent[]) {
    gboolean visited[MAX_NODES] = {FALSE};
    
    for (int i = 0; i < app->num_points; i++) {
        dist[i] = INF;
        parent[i] = -1;
    }
    
    dist[src] = 0;
    
    for (int count = 0; count < app->num_points - 1; count++) {
        int min_dist = INF, u = -1;
        
        for (int v = 0; v < app->num_points; v++) {
            if (!visited[v] && dist[v] <= min_dist) {
                min_dist = dist[v];
                u = v;
            }
        }
        
        if (u == -1) break;
        visited[u] = TRUE;
        
        for (int v = 0; v < app->num_points; v++) {
            if (!visited[v] && graph[u][v] && dist[u] != INF &&
                dist[u] + graph[u][v] < dist[v]) {
                dist[v] = dist[u] + graph[u][v];
                parent[v] = u;
            }
        }
    }
}

int knapsack(int capacity) {
    int dp[capacity + 1];
    memset(dp, 0, sizeof(dp));
    for (int i = 0; i < app->num_packages; i++) {
        for (int w = capacity; w >= app->packages[i].weight; w--) {
            int value_with_priority = app->packages[i].value; // No priority
            int carbon_penalty = (int)(app->packages[i].carbon_footprint * 10);
            int net_value = value_with_priority - carbon_penalty;
            if (dp[w - app->packages[i].weight] + net_value > dp[w]) {
                dp[w] = dp[w - app->packages[i].weight] + net_value;
            }
        }
    }
    return dp[capacity];
}

double calculate_distance(DeliveryPoint *p1, DeliveryPoint *p2) {
    double dx = p1->x - p2->x;
    double dy = p1->y - p2->y;
    return sqrt(dx * dx + dy * dy);
}

void initialize_data() {
    char *locations[] = {
        "Depot - Koramangala", "Electronic City", "Whitefield", "Banashankari",
        "Jayanagar", "Indiranagar", "HSR Layout", "BTM Layout",
        "Malleshwaram", "Rajajinagar", "Hebbal", "Marathahalli",
        "Yeshwanthpur", "Kengeri", "Vijayanagar", "Ulsoor",
        "Shivajinagar", "KR Puram", "Basavanagudi", "RT Nagar"
    };
    
    app->num_points = 18;
    
    for (int i = 0; i < app->num_points; i++) {
        app->points[i].id = i;
        strcpy(app->points[i].name, locations[i]);
        app->points[i].is_depot = (i == 0);
        app->points[i].x = 100 + (i % 6) * 160 + (rand() % 20);
        app->points[i].y = 100 + (i / 6) * 160 + (rand() % 20);
        app->points[i].package_count = rand() % 5 + 1;
    }
    
    app->num_packages = 20;
    for (int i = 0; i < app->num_packages; i++) {
        app->packages[i].id = i;
        app->packages[i].weight = rand() % 10 + 1;
        app->packages[i].value = rand() % 100 + 50;
        app->packages[i].priority = rand() % 3 + 1;
        app->packages[i].carbon_footprint = (rand() % 50 + 10) / 10.0;
        app->packages[i].destination_id = rand() % (app->num_points - 1) + 1;
    }
    
    app->num_routes = 0;
    for (int i = 0; i < app->num_points; i++) {
        for (int j = i + 1; j < app->num_points; j++) {
            app->routes[app->num_routes].from = i;
            app->routes[app->num_routes].to = j;
            app->routes[app->num_routes].distance = calculate_distance(&app->points[i], &app->points[j]);
            app->routes[app->num_routes].carbon_emission_factor = 0.21 + (rand() % 10) / 100.0;
            app->num_routes++;
        }
    }
}

void calculate_optimal_route() {
    if (app->optimal_route) free(app->optimal_route);
    
    app->optimal_route = malloc(app->num_points * sizeof(int));
    app->route_length = 0;
    app->total_distance = 0;
    app->total_emissions = 0;
    
    gboolean visited[MAX_NODES] = {FALSE};
    int current = 0;
    app->optimal_route[app->route_length++] = current;
    visited[current] = TRUE;
    
    while (app->route_length < app->num_points) {
        double min_distance = INF;
        int next_point = -1;
        
        for (int i = 0; i < app->num_points; i++) {
            if (!visited[i]) {
                double dist = calculate_distance(&app->points[current], &app->points[i]);
                if (dist < min_distance) {
                    min_distance = dist;
                    next_point = i;
                }
            }
        }
        
        if (next_point != -1) {
            app->optimal_route[app->route_length++] = next_point;
            visited[next_point] = TRUE;
            app->total_distance += min_distance;
            app->total_emissions += min_distance * 0.21;
            current = next_point;
        } else {
            break;
        }
    }
    
    if (app->route_length > 1) {
        app->total_distance += calculate_distance(&app->points[current], &app->points[0]);
        app->total_emissions += calculate_distance(&app->points[current], &app->points[0]) * 0.21;
    }
    
    app->show_route = TRUE;
}

gboolean on_draw(GtkWidget *widget, cairo_t *cr, gpointer data) {
    cairo_set_source_rgb(cr, 0.95, 0.95, 0.95);
    cairo_paint(cr);
    // Draw all routes and distances (gray)
    cairo_set_source_rgb(cr, 0.7, 0.7, 0.7);
    cairo_set_line_width(cr, 1.0);
    for (int i = 0; i < app->num_routes; i++) {
        int from = app->routes[i].from;
        int to = app->routes[i].to;
        double x1 = app->points[from].x;
        double y1 = app->points[from].y;
        double x2 = app->points[to].x;
        double y2 = app->points[to].y;
        cairo_move_to(cr, x1, y1);
        cairo_line_to(cr, x2, y2);
        cairo_stroke(cr);
        // Draw distance at midpoint
        double mx = (x1 + x2) / 2;
        double my = (y1 + y2) / 2;
        char dist_str[16];
        snprintf(dist_str, sizeof(dist_str), "%.1f", app->routes[i].distance / 10.0); // scale for km
        cairo_set_source_rgb(cr, 0.2, 0.2, 0.2);
        cairo_move_to(cr, mx + 5, my - 5);
        cairo_show_text(cr, dist_str);
        cairo_set_source_rgb(cr, 0.7, 0.7, 0.7); // reset for next line
    }
    // Highlight the optimal route (blue, thick)
    if (app->show_route && app->optimal_route && app->route_length > 1) {
        cairo_set_source_rgb(cr, 0.2, 0.6, 1.0);
        cairo_set_line_width(cr, 3.0);
        for (int i = 0; i < app->route_length - 1; i++) {
            int from = app->optimal_route[i];
            int to = app->optimal_route[i + 1];
            cairo_move_to(cr, app->points[from].x, app->points[from].y);
            cairo_line_to(cr, app->points[to].x, app->points[to].y);
            cairo_stroke(cr);
        }
        // Close the loop to depot if needed
        int last = app->optimal_route[app->route_length - 1];
        cairo_move_to(cr, app->points[last].x, app->points[last].y);
        cairo_line_to(cr, app->points[0].x, app->points[0].y);
        cairo_stroke(cr);
    }
    
    for (int i = 0; i < app->num_points; i++) {
        double x = app->points[i].x;
        double y = app->points[i].y;
        
        if (app->points[i].is_depot) {
            cairo_set_source_rgb(cr, 1.0, 0.0, 0.0);
            cairo_arc(cr, x, y, 12, 0, 2 * M_PI);
        } else {
            cairo_set_source_rgb(cr, 0.0, 0.8, 0.0);
            cairo_arc(cr, x, y, 10, 0, 2 * M_PI);
        }
        cairo_fill(cr);
        
        if (i == app->selected_point) {
            cairo_set_source_rgb(cr, 1.0, 1.0, 0.0);
            cairo_set_line_width(cr, 2.0);
            cairo_arc(cr, x, y, 15, 0, 2 * M_PI);
            cairo_stroke(cr);
        }
        
        cairo_set_source_rgb(cr, 0.0, 0.0, 0.0);
        cairo_move_to(cr, x + 15, y - 5);
        cairo_show_text(cr, app->points[i].name);
        
        char package_info[50];
        snprintf(package_info, sizeof(package_info), "Packages: %d", app->points[i].package_count);
        cairo_move_to(cr, x + 15, y + 10);
        cairo_show_text(cr, package_info);
        // Show total value of packages for this node
        int total_value = 0;
        for (int j = 0; j < app->num_packages; j++) {
            if (app->packages[j].destination_id == app->points[i].id) {
                total_value += app->packages[j].value - (int)(app->packages[j].carbon_footprint * 10);
            }
        }
        if (total_value > 0) {
            char value_info[50];
            snprintf(value_info, sizeof(value_info), "Value: %d", total_value);
            cairo_move_to(cr, x + 15, y + 25);
            cairo_show_text(cr, value_info);
        }
    }
    
    if (app->animation_running && app->optimal_route) {
        if (app->current_route_segment < app->route_length) {
            int from = app->optimal_route[app->current_route_segment];
            int to = app->optimal_route[(app->current_route_segment + 1) % app->route_length];
            
            double start_x = app->points[from].x;
            double start_y = app->points[from].y;
            double end_x = app->points[to].x;
            double end_y = app->points[to].y;
            
            double vehicle_x = start_x + (end_x - start_x) * app->vehicle_progress;
            double vehicle_y = start_y + (end_y - start_y) * app->vehicle_progress;
            
            cairo_set_source_rgb(cr, 0.8, 0.2, 0.2);
            cairo_arc(cr, vehicle_x, vehicle_y, VEHICLE_SIZE/2, 0, 2 * M_PI);
            cairo_fill(cr);
            
            cairo_set_source_rgb(cr, 1.0, 1.0, 1.0);
            cairo_arc(cr, 
                vehicle_x + VEHICLE_SIZE/3 * cos(atan2(end_y - start_y, end_x - start_x)),
                vehicle_y + VEHICLE_SIZE/3 * sin(atan2(end_y - start_y, end_x - start_x)),
                VEHICLE_SIZE/4, 0, 2 * M_PI);
            cairo_fill(cr);
        }
    }
    
    return FALSE;
}

gboolean animation_step(gpointer data) {
    DeliveryApp *app = (DeliveryApp *)data;
    
    if (app->current_route_segment >= app->route_length) {
        app->current_route_segment = 0;
        app->vehicle_progress = 0.0;
        return G_SOURCE_CONTINUE;
    }

    app->vehicle_progress += 0.02 * app->animation_speed;
    
    if (app->vehicle_progress >= 1.0) {
        app->current_route_segment++;
        app->vehicle_progress = 0.0;
        
        if (app->current_route_segment >= app->route_length) {
            app->current_route_segment = 0;
        }
    }
    
    gtk_widget_queue_draw(app->drawing_area);
    return G_SOURCE_CONTINUE;
}

void start_delivery_animation(DeliveryApp *app) {
    if (!app->animation_running && app->optimal_route) {
        app->animation_running = TRUE;
        app->current_route_segment = 0;
        app->vehicle_progress = 0.0;
        app->animation_timeout_id = g_timeout_add(ANIMATION_INTERVAL, animation_step, app);
    }
}

void stop_delivery_animation(DeliveryApp *app) {
    if (app->animation_running) {
        app->animation_running = FALSE;
        if (app->animation_timeout_id) {
            g_source_remove(app->animation_timeout_id);
            app->animation_timeout_id = 0;
        }
    }
}

void on_start_delivery(GtkWidget *widget, gpointer data) {
    DeliveryApp *app = (DeliveryApp *)data;
    
    if (!app->animation_running) {
        start_delivery_animation(app);
        gtk_button_set_label(GTK_BUTTON(widget), "Stop Delivery");
    } else {
        stop_delivery_animation(app);
        gtk_button_set_label(GTK_BUTTON(widget), "Start Delivery");
    }
}

void on_speed_changed(GtkRange *range, gpointer data) {
    DeliveryApp *app = (DeliveryApp *)data;
    app->animation_speed = gtk_range_get_value(range);
}

gboolean on_button_press(GtkWidget *widget, GdkEventButton *event, gpointer data) {
    double x = event->x;
    double y = event->y;
    
    app->selected_point = -1;
    for (int i = 0; i < app->num_points; i++) {
        double dx = x - app->points[i].x;
        double dy = y - app->points[i].y;
        double distance = sqrt(dx * dx + dy * dy);
        
        if (distance <= 15) {
            app->selected_point = i;
            break;
        }
    }
    
    if (app->selected_point >= 0) {
        char info[500];
        DeliveryPoint *point = &app->points[app->selected_point];
        snprintf(info, sizeof(info), 
                "Selected: %s | Packages: %d | Type: %s",
                point->name, point->package_count,
                point->is_depot ? "Depot" : "Delivery Point");
        gtk_label_set_text(GTK_LABEL(app->info_label), info);
    } else {
        gtk_label_set_text(GTK_LABEL(app->info_label), "Click on a delivery point for details");
    }
    
    gtk_widget_queue_draw(widget);
    return TRUE;
}

void on_calculate_route(GtkWidget *widget, gpointer data) {
    calculate_optimal_route();
    
    char info[300];
    int optimal_value = knapsack(50);
    snprintf(info, sizeof(info),
            "Route calculated! Distance: %.1f km | Emissions: %.2f kg CO2 | Package Value: %d",
            app->total_distance / 10, app->total_emissions / 10, optimal_value);
    gtk_label_set_text(GTK_LABEL(app->info_label), info);
    
    gtk_widget_queue_draw(app->drawing_area);
}

// Helper to show selected packages for knapsack
void show_knapsack_details(int capacity) {
    int dp[capacity + 1];
    int keep[MAX_PACKAGES][capacity + 1];
    memset(dp, 0, sizeof(dp));
    memset(keep, 0, sizeof(keep));
    for (int i = 0; i < app->num_packages; i++) {
        for (int w = capacity; w >= app->packages[i].weight; w--) {
            int value_with_priority = app->packages[i].value; // No priority
            int carbon_penalty = (int)(app->packages[i].carbon_footprint * 10);
            int net_value = value_with_priority - carbon_penalty;
            if (dp[w - app->packages[i].weight] + net_value > dp[w]) {
                dp[w] = dp[w - app->packages[i].weight] + net_value;
                keep[i][w] = 1;
            }
        }
    }
    // Backtrack to find selected packages
    int w = capacity;
    int total_weight = 0;
    int total_value = 0;
    char details[4096] = "";
    strcat(details, "ID   Wt   Val   C.Foot  Location                Value\n");
    for (int i = app->num_packages - 1; i >= 0; i--) {
        if (w >= app->packages[i].weight && keep[i][w]) {
            int net_value = app->packages[i].value - (int)(app->packages[i].carbon_footprint * 10);
            char pkg[256];
            snprintf(pkg, sizeof(pkg), "%2d  %3d  %4d   %5.1f  %-22s %5d\n",
                app->packages[i].id, app->packages[i].weight, app->packages[i].value, app->packages[i].carbon_footprint,
                app->points[app->packages[i].destination_id].name, net_value);
            strcat(details, pkg);
            total_weight += app->packages[i].weight;
            total_value += net_value;
            w -= app->packages[i].weight;
        }
    }
    char info[5000];
    snprintf(info, sizeof(info),
        "Knapsack: MaxCap=%dkg | TotalValue=%d | TotalWeight=%dkg\n-----------------------------------------------\n%s",
        capacity, total_value, total_weight, details);
    // Set monospace font for the table label
    PangoAttrList *attrs = pango_attr_list_new();
    pango_attr_list_insert(attrs, pango_attr_family_new("monospace"));
    gtk_label_set_attributes(GTK_LABEL(app->table_label), attrs);
    pango_attr_list_unref(attrs);
    gtk_label_set_text(GTK_LABEL(app->table_label), info);
}

void on_optimize_packages(GtkWidget *widget, gpointer data) {
    int max_capacity = 50;
    show_knapsack_details(max_capacity);
    gtk_widget_queue_draw(app->drawing_area);
}

void init_gui() {
    app->window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title(GTK_WINDOW(app->window), "Bengaluru Smart Delivery System");
    gtk_window_set_default_size(GTK_WINDOW(app->window), 900, 700);
    gtk_window_set_position(GTK_WINDOW(app->window), GTK_WIN_POS_CENTER);
    GtkWidget *vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 5);
    gtk_container_add(GTK_CONTAINER(app->window), vbox);
    GtkWidget *hbox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 5);
    gtk_box_pack_start(GTK_BOX(vbox), hbox, FALSE, FALSE, 5);
    app->route_button = gtk_button_new_with_label("Calculate Optimal Route");
    app->optimize_button = gtk_button_new_with_label("Optimize Packages");
    app->start_button = gtk_button_new_with_label("Start Delivery");
    GtkWidget *speed_label = gtk_label_new("Speed:");
    app->speed_scale = gtk_scale_new_with_range(GTK_ORIENTATION_HORIZONTAL, 0.5, 3.0, 0.5);
    gtk_range_set_value(GTK_RANGE(app->speed_scale), 1.0);
    gtk_box_pack_start(GTK_BOX(hbox), app->route_button, FALSE, FALSE, 5);
    gtk_box_pack_start(GTK_BOX(hbox), app->optimize_button, FALSE, FALSE, 5);
    gtk_box_pack_start(GTK_BOX(hbox), app->start_button, FALSE, FALSE, 5);
    gtk_box_pack_start(GTK_BOX(hbox), speed_label, FALSE, FALSE, 5);
    gtk_box_pack_start(GTK_BOX(hbox), app->speed_scale, FALSE, FALSE, 5);
    // --- Main horizontal box for graph and table ---
    GtkWidget *hbox_main = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 10);
    app->drawing_area = gtk_drawing_area_new();
    gtk_widget_set_size_request(app->drawing_area, CANVAS_WIDTH, CANVAS_HEIGHT);
    gtk_box_pack_start(GTK_BOX(hbox_main), app->drawing_area, FALSE, FALSE, 0);
    app->table_label = gtk_label_new("");
    gtk_label_set_xalign(GTK_LABEL(app->table_label), 0.0);
    gtk_label_set_yalign(GTK_LABEL(app->table_label), 0.0);
    gtk_widget_set_halign(app->table_label, GTK_ALIGN_START);
    gtk_widget_set_valign(app->table_label, GTK_ALIGN_START);
    gtk_box_pack_start(GTK_BOX(hbox_main), app->table_label, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(vbox), hbox_main, TRUE, TRUE, 0);
    app->info_label = gtk_label_new("Bengaluru Smart Delivery System - Click on delivery points for details");
    gtk_box_pack_start(GTK_BOX(vbox), app->info_label, FALSE, FALSE, 5);
    g_signal_connect(app->window, "destroy", G_CALLBACK(gtk_main_quit), NULL);
    g_signal_connect(app->drawing_area, "draw", G_CALLBACK(on_draw), NULL);
    g_signal_connect(app->drawing_area, "button-press-event", G_CALLBACK(on_button_press), NULL);
    g_signal_connect(app->route_button, "clicked", G_CALLBACK(on_calculate_route), NULL);
    g_signal_connect(app->optimize_button, "clicked", G_CALLBACK(on_optimize_packages), NULL);
    g_signal_connect(app->start_button, "clicked", G_CALLBACK(on_start_delivery), app);
    g_signal_connect(app->speed_scale, "value-changed", G_CALLBACK(on_speed_changed), app);
    gtk_widget_add_events(app->drawing_area, GDK_BUTTON_PRESS_MASK);
    app->animation_running = FALSE;
    app->current_route_segment = 0;
    app->vehicle_progress = 0.0;
    app->animation_speed = 1.0;
    gtk_widget_show_all(app->window);
}

int main(int argc, char *argv[]) {
    gtk_init(&argc, &argv);
    
    app = malloc(sizeof(DeliveryApp));
    memset(app, 0, sizeof(DeliveryApp));
    app->selected_point = -1;
    app->show_route = FALSE;
    
    initialize_data();
    init_gui();
    
    printf("Bengaluru Smart Delivery System Started!\n");
    printf("- Click on delivery points for details\n");
    printf("- Use 'Calculate Optimal Route' to find best path\n");
    printf("- Use 'Start Delivery' to begin vehicle animation\n");
    
    gtk_main();
    
    if (app->optimal_route) free(app->optimal_route);
    free(app);
    
    return 0;
}

