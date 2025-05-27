#include <gtk/gtk.h>
#include <cairo.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <limits.h>

#define MAX_NODES 50
#define MAX_PACKAGES 100
#define INF 999999
#define CANVAS_WIDTH 800
#define CANVAS_HEIGHT 600

// Structure for delivery locations
typedef struct {
    int id;
    char name[100];
    double lat, lon;
    double x, y; // Screen coordinates
    int is_depot;
    int package_count;
} DeliveryPoint;

// Structure for packages
typedef struct {
    int id;
    int weight;
    int value;
    int priority;
    double carbon_footprint;
    int destination_id;
} Package;

// Structure for routes
typedef struct {
    int from, to;
    double distance;
    double carbon_emission_factor;
} Route;

// Main application structure
typedef struct {
    GtkWidget *window;
    GtkWidget *drawing_area;
    GtkWidget *info_label;
    GtkWidget *route_button;
    GtkWidget *optimize_button;
    
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
} DeliveryApp;

// Global app instance
DeliveryApp *app;

// Dijkstra's algorithm implementation
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

// Knapsack algorithm for package optimization
int knapsack(int capacity) {
    int dp[capacity + 1];
    memset(dp, 0, sizeof(dp));
    
    for (int i = 0; i < app->num_packages; i++) {
        for (int w = capacity; w >= app->packages[i].weight; w--) {
            int value_with_priority = app->packages[i].value * app->packages[i].priority;
            int carbon_penalty = (int)(app->packages[i].carbon_footprint * 10);
            int net_value = value_with_priority - carbon_penalty;
            
            if (dp[w - app->packages[i].weight] + net_value > dp[w]) {
                dp[w] = dp[w - app->packages[i].weight] + net_value;
            }
        }
    }
    
    return dp[capacity];
}

// Calculate distance between two points
double calculate_distance(DeliveryPoint *p1, DeliveryPoint *p2) {
    double dx = p1->x - p2->x;
    double dy = p1->y - p2->y;
    return sqrt(dx * dx + dy * dy);
}

// Initialize sample data for Bengaluru
void initialize_data() {
    // Sample delivery points in Bengaluru
    char *locations[] = {
        "Depot - Koramangala", "Electronic City", "Whitefield", "Banashankari",
        "Jayanagar", "Indiranagar", "HSR Layout", "BTM Layout",
        "Malleshwaram", "Rajajinagar", "Hebbal", "Marathahalli"
    };
    
    app->num_points = 12;
    
    // Initialize points with pseudo-coordinates
    for (int i = 0; i < app->num_points; i++) {
        app->points[i].id = i;
        strcpy(app->points[i].name, locations[i]);
        app->points[i].is_depot = (i == 0);
        
        // Distribute points across the canvas
        app->points[i].x = 100 + (i % 4) * 180 + (rand() % 50);
        app->points[i].y = 100 + (i / 4) * 150 + (rand() % 50);
        
        app->points[i].package_count = rand() % 5 + 1;
    }
    
    // Initialize sample packages
    app->num_packages = 20;
    for (int i = 0; i < app->num_packages; i++) {
        app->packages[i].id = i;
        app->packages[i].weight = rand() % 10 + 1;
        app->packages[i].value = rand() % 100 + 50;
        app->packages[i].priority = rand() % 3 + 1;
        app->packages[i].carbon_footprint = (rand() % 50 + 10) / 10.0;
        app->packages[i].destination_id = rand() % (app->num_points - 1) + 1;
    }
    
    // Initialize routes
    app->num_routes = 0;
    for (int i = 0; i < app->num_points; i++) {
        for (int j = i + 1; j < app->num_points; j++) {
            app->routes[app->num_routes].from = i;
            app->routes[app->num_routes].to = j;
            app->routes[app->num_routes].distance = calculate_distance(&app->points[i], &app->points[j]);
            app->routes[app->num_routes].carbon_emission_factor = 0.21 + (rand() % 10) / 100.0; // kg CO2 per km
            app->num_routes++;
        }
    }
}

// Calculate optimal route using nearest neighbor with Dijkstra
void calculate_optimal_route() {
    if (app->optimal_route) {
        free(app->optimal_route);
    }
    
    app->optimal_route = malloc(app->num_points * sizeof(int));
    app->route_length = 0;
    app->total_distance = 0;
    app->total_emissions = 0;
    
    gboolean visited[MAX_NODES] = {FALSE};
    int current = 0; // Start from depot
    app->optimal_route[app->route_length++] = current;
    visited[current] = TRUE;
    
    // Nearest neighbor approach
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
            app->total_emissions += min_distance * 0.21; // Approximate emission factor
            current = next_point;
        } else {
            break;
        }
    }
    
    // Return to depot
    if (app->route_length > 1) {
        app->total_distance += calculate_distance(&app->points[current], &app->points[0]);
        app->total_emissions += calculate_distance(&app->points[current], &app->points[0]) * 0.21;
    }
    
    app->show_route = TRUE;
}

// Drawing callback
gboolean on_draw(GtkWidget *widget, cairo_t *cr, gpointer data) {
    // Clear background
    cairo_set_source_rgb(cr, 0.95, 0.95, 0.95);
    cairo_paint(cr);
    
    // Draw routes if showing optimal route
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
        
        // Return to depot
        if (app->route_length > 1) {
            int last = app->optimal_route[app->route_length - 1];
            cairo_move_to(cr, app->points[last].x, app->points[last].y);
            cairo_line_to(cr, app->points[0].x, app->points[0].y);
            cairo_stroke(cr);
        }
    }
    
    // Draw delivery points
    for (int i = 0; i < app->num_points; i++) {
        double x = app->points[i].x;
        double y = app->points[i].y;
        
        // Draw point
        if (app->points[i].is_depot) {
            cairo_set_source_rgb(cr, 1.0, 0.0, 0.0); // Red for depot
            cairo_arc(cr, x, y, 12, 0, 2 * M_PI);
        } else {
            cairo_set_source_rgb(cr, 0.0, 0.8, 0.0); // Green for delivery points
            cairo_arc(cr, x, y, 10, 0, 2 * M_PI);
        }
        cairo_fill(cr);
        
        // Draw selection highlight
        if (i == app->selected_point) {
            cairo_set_source_rgb(cr, 1.0, 1.0, 0.0);
            cairo_set_line_width(cr, 2.0);
            cairo_arc(cr, x, y, 15, 0, 2 * M_PI);
            cairo_stroke(cr);
        }
        
        // Draw labels
        cairo_set_source_rgb(cr, 0.0, 0.0, 0.0);
        cairo_move_to(cr, x + 15, y - 5);
        cairo_show_text(cr, app->points[i].name);
        
        // Show package count
        char package_info[50];
        snprintf(package_info, sizeof(package_info), "Packages: %d", app->points[i].package_count);
        cairo_move_to(cr, x + 15, y + 10);
        cairo_show_text(cr, package_info);
    }
    
    return FALSE;
}

// Mouse click callback
gboolean on_button_press(GtkWidget *widget, GdkEventButton *event, gpointer data) {
    double x = event->x;
    double y = event->y;
    
    // Find clicked point
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
    
    // Update info label
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

// Route calculation button callback
void on_calculate_route(GtkWidget *widget, gpointer data) {
    calculate_optimal_route();
    
    // Update info display
    char info[300];
    int optimal_value = knapsack(50); // Assuming 50kg capacity
    snprintf(info, sizeof(info),
            "Route calculated! Distance: %.1f km | Emissions: %.2f kg CO2 | Package Value: %d",
            app->total_distance / 10, app->total_emissions / 10, optimal_value);
    gtk_label_set_text(GTK_LABEL(app->info_label), info);
    
    gtk_widget_queue_draw(app->drawing_area);
}

// Package optimization callback
void on_optimize_packages(GtkWidget *widget, gpointer data) {
    int optimal_value = knapsack(50); // 50kg capacity
    
    char info[200];
    snprintf(info, sizeof(info),
            "Package optimization complete! Optimal value: %d (50kg capacity)",
            optimal_value);
    gtk_label_set_text(GTK_LABEL(app->info_label), info);
}

// Initialize GTK interface
void init_gui() {
    app->window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title(GTK_WINDOW(app->window), "Bengaluru Smart Delivery System");
    gtk_window_set_default_size(GTK_WINDOW(app->window), 900, 700);
    gtk_window_set_position(GTK_WINDOW(app->window), GTK_WIN_POS_CENTER);
    
    // Main container
    GtkWidget *vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 5);
    gtk_container_add(GTK_CONTAINER(app->window), vbox);
    
    // Control panel
    GtkWidget *hbox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 5);
    gtk_box_pack_start(GTK_BOX(vbox), hbox, FALSE, FALSE, 5);
    
    app->route_button = gtk_button_new_with_label("Calculate Optimal Route");
    app->optimize_button = gtk_button_new_with_label("Optimize Packages");
    
    gtk_box_pack_start(GTK_BOX(hbox), app->route_button, FALSE, FALSE, 5);
    gtk_box_pack_start(GTK_BOX(hbox), app->optimize_button, FALSE, FALSE, 5);
    
    // Drawing area
    app->drawing_area = gtk_drawing_area_new();
    gtk_widget_set_size_request(app->drawing_area, CANVAS_WIDTH, CANVAS_HEIGHT);
    gtk_box_pack_start(GTK_BOX(vbox), app->drawing_area, TRUE, TRUE, 0);
    
    // Info label
    app->info_label = gtk_label_new("Bengaluru Smart Delivery System - Click on delivery points for details");
    gtk_box_pack_start(GTK_BOX(vbox), app->info_label, FALSE, FALSE, 5);
    
    // Connect signals
    g_signal_connect(app->window, "destroy", G_CALLBACK(gtk_main_quit), NULL);
    g_signal_connect(app->drawing_area, "draw", G_CALLBACK(on_draw), NULL);
    g_signal_connect(app->drawing_area, "button-press-event", G_CALLBACK(on_button_press), NULL);
    g_signal_connect(app->route_button, "clicked", G_CALLBACK(on_calculate_route), NULL);
    g_signal_connect(app->optimize_button, "clicked", G_CALLBACK(on_optimize_packages), NULL);
    
    // Enable events
    gtk_widget_add_events(app->drawing_area, GDK_BUTTON_PRESS_MASK);
    
    gtk_widget_show_all(app->window);
}

int main(int argc, char *argv[]) {
    // Initialize GTK
    gtk_init(&argc, &argv);
    
    // Initialize application
    app = malloc(sizeof(DeliveryApp));
    memset(app, 0, sizeof(DeliveryApp));
    app->selected_point = -1;
    app->show_route = FALSE;
    
    // Initialize data and GUI
    initialize_data();
    init_gui();
    
    printf("Bengaluru Smart Delivery System Started!\n");
    printf("- Click on delivery points for details\n");
    printf("- Use 'Calculate Optimal Route' to find best path\n");
    printf("- Use 'Optimize Packages' for load optimization\n");
    
    // Run main loop
    gtk_main();
    
    // Cleanup
    if (app->optimal_route) {
        free(app->optimal_route);
    }
    free(app);
    
    return 0;
}

/*
=== WINDOWS COMPILATION INSTRUCTIONS ===

Method 1 - MSYS2 (Recommended):
1. Install MSYS2 from https://www.msys2.org/
2. In MSYS2 terminal:
   pacman -S mingw-w64-x86_64-gcc mingw-w64-x86_64-gtk3 mingw-w64-x86_64-pkg-config
3. Compile: gcc -o delivery_system.exe delivery_system.c `pkg-config --cflags --libs gtk+-3.0` -lm
4. Run: ./delivery_system.exe

Method 2 - Visual Studio:
1. Install Visual Studio with C++ tools
2. Install vcpkg and GTK: vcpkg install gtk:x64-windows
3. Create project and link GTK libraries

Method 3 - Pre-built GTK:
1. Download GTK for Windows from gtk.org
2. Add GTK bin directory to PATH
3. Use MinGW or Visual Studio to compile with GTK flags

Required Libraries: gtk+-3.0, cairo, glib-2.0, gobject-2.0
*/