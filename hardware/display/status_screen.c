#include <zephyr/kernel.h>
#include <stdio.h>

#include <zmk/battery.h>
#include <zmk/display.h>
#include <zmk/display/status_screen.h>
#include <zmk/display/widgets/layer_status.h>
#include <zmk/display/widgets/output_status.h>
#include <zmk/event_manager.h>
#include <zmk/events/battery_state_changed.h>
#include <zmk/events/usb_conn_state_changed.h>
#include <zmk/usb.h>
#include <zmk/split/central.h>

static struct zmk_widget_output_status output_widget;
static struct zmk_widget_layer_status layer_widget;
static lv_obj_t *battery_label;
struct battery_status_state {
    uint8_t left;
    uint8_t right;
    bool usb_present;
};

static struct battery_status_state battery_get_state(const zmk_event_t *eh) {
    struct battery_status_state state = {
        .left = zmk_battery_state_of_charge(),
        .usb_present = zmk_usb_is_powered(),
    };
    /* Toucan has one peripheral: the right half, split source 0. */
    zmk_split_central_get_peripheral_battery_level(0, &state.right);
    return state;
}

static void update_batteries(struct battery_status_state state) {
    char text[32];
    snprintf(text, sizeof(text), "L %s%u%%\nR %u%%",
             state.usb_present ? LV_SYMBOL_CHARGE " " : "", state.left, state.right);
    lv_label_set_text(battery_label, text);
}

ZMK_DISPLAY_WIDGET_LISTENER(toucan_batteries, struct battery_status_state, update_batteries,
                            battery_get_state)
ZMK_SUBSCRIPTION(toucan_batteries, zmk_battery_state_changed);
ZMK_SUBSCRIPTION(toucan_batteries, zmk_peripheral_battery_state_changed);
ZMK_SUBSCRIPTION(toucan_batteries, zmk_usb_conn_state_changed);

lv_obj_t *zmk_display_status_screen(void) {
    lv_obj_t *screen = lv_obj_create(NULL);

    zmk_widget_output_status_init(&output_widget, screen);
    lv_obj_align(zmk_widget_output_status_obj(&output_widget), LV_ALIGN_TOP_LEFT, 0, 0);

    battery_label = lv_label_create(screen);
    lv_obj_set_style_text_font(battery_label, lv_theme_get_font_small(screen), LV_PART_MAIN);
    lv_obj_set_style_text_align(battery_label, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);
    toucan_batteries_init();
    lv_obj_align(battery_label, LV_ALIGN_TOP_RIGHT, 0, 0);

    zmk_widget_layer_status_init(&layer_widget, screen);
    lv_obj_set_style_text_font(zmk_widget_layer_status_obj(&layer_widget),
                               lv_theme_get_font_small(screen), LV_PART_MAIN);
    lv_obj_align(zmk_widget_layer_status_obj(&layer_widget), LV_ALIGN_BOTTOM_LEFT, 0, 0);

    return screen;
}
