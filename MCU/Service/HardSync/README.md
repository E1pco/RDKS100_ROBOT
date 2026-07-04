# Hard Sync Service

This service validates the configured outputs, asks the port layer to configure
periodic outputs, and starts/stops all enabled channels together.

The service does not directly access LPWM, GPIO, trigger-bus, or timer registers.
