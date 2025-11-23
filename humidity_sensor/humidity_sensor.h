typedef void (* get_humidity_process_t)(float h);

int start_humidity_sensor(get_humidity_process_t humidity_process);
