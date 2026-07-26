
/* -----------------------------------------------------------------------
 * Internal listener list — grows by doubling, never shrinks.
 * --------------------------------------------------------------------- */

#define INITIAL_CAPACITY 8

static ESPEventCallback *s_listeners = NULL;
static int s_count    = 0;
static int s_capacity = 0;

static int grow(void)
{
    int new_cap = (s_capacity == 0) ? INITIAL_CAPACITY : s_capacity * 2;
    ESPEventCallback *tmp = realloc(s_listeners, new_cap * sizeof(ESPEventCallback));
    if (!tmp) return 0;
    s_listeners = tmp;
    s_capacity  = new_cap;
    return 1;
}

void esp_add_event_listener(ESPEventCallback listener)
{
    int i;
    if (!listener) return;

    /* duplicate guard */
    for (i = 0; i < s_count; i++)
        if (s_listeners[i] == listener) return;

    if (s_count == s_capacity && !grow()) return;  /* OOM — skip silently */

    s_listeners[s_count++] = listener;
}

void esp_remove_event_listener(ESPEventCallback listener)
{
    int i;
    if (!listener) return;
    for (i = 0; i < s_count; i++) {
        if (s_listeners[i] == listener) {
            /* fill gap by shifting tail left by one */
            memmove(&s_listeners[i], &s_listeners[i + 1],
                    (s_count - i - 1) * sizeof(ESPEventCallback));
            s_count--;
            return;
        }
    }
}

void esp_fire_event(const ESPEvent *event)
{
    int i;
    if (!event) return;
    /* iterate over a snapshot count so listeners added during delivery
     * do not receive the current event */
    int count = s_count;
    for (i = 0; i < count; i++)
        s_listeners[i](event);
}

void esp_fire_event_type(ESPEventType type)
{
    ESPEvent e;
    memset(&e, 0, sizeof(e));
    e.type = type;
    esp_fire_event(&e);
}
void fireResizeEvent()
{
    ESPEvent e;
    e.type = ESP_EVENT_SIZE_CHANGED;
    esp_fire_event(&e);
}
void fireInitEvent()
{
    ESPEvent e;
    e.type = ESP_EVENT_INIT;
    esp_fire_event(&e);
}
void fireKillEvent()
{
    ESPEvent e;
    e.type = ESP_EVENT_KILL;
    esp_fire_event(&e);
}

void esp_clear_event_listeners(void)
{
    s_count = 0;
    /* keep allocation; just reset count */
}

