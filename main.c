#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <gpiod.h>
#include <unistd.h>
#include <assert.h>

int main()
{
    char *chipname = "gpiochip0";
    unsigned int ln = 27;
    struct timespec last = { 0, 0 };
    struct timespec ts = { 1, 0 };
    struct gpiod_line_event event;
    struct gpiod_chip *chip;
    struct gpiod_line *line;
    int r,n;

    chip = gpiod_chip_open_by_name(chipname);
    assert(chip);

    line = gpiod_chip_get_line(chip, ln);
    assert(line);

    r = gpiod_line_request_falling_edge_events(line, "event27");
    assert(r>=0);

    while (1)
    {
        r = gpiod_line_event_wait(line, &ts);
        if (!r) continue;
        assert(r>0);
        
        r = gpiod_line_event_read(line, &event);
        assert (r>=0);
        
        // Debounce
        int n = (event.ts.tv_nsec - last.tv_nsec)/1000000 + (event.ts.tv_sec - last.tv_sec) * 1000;
        if (n < 80) continue;
        
        last = event.ts;
        printf("event %d.%09d %4d\n", event.ts.tv_sec, event.ts.tv_nsec, n);

    }
    return 0;
}

