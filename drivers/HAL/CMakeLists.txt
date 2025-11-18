idf_component_register(
    SRCS "phases.cpp"
    INCLUDE_DIRS "."
    PRIV_INCLUDE_DIRS "../"
    REQUIRES esp_timer driver freertos log
)
