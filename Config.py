### IMPORTS
import SpecialMessages, json

### CLASS
class Config:
    
    _def_config = {
        "mqtt_base_topic":  "irrigation",
        "mqtt_server":      "192.168.1.30",
        "mqtt_port":        1883,
        "serial_port":      "/dev/ttyACM1",
        "serial_baud":      9600,
        "log_level":        5
    }
    
    # This will be used after getting config data
    _config = {}
    
    _msg = SpecialMessages.SpecialMessages(_def_config["log_level"])
    
    def __init__(self, path):
        try:
            with open(path) as json_data:
                d = json.load(json_data)
                
                self._override("mqtt_base_topic", d)
                self._override("mqtt_server", d)
                self._override("mqtt_port", d)
                self._override("serial_port", d)
                self._override("serial_baud", d)
                self._override("log_level", d)
                
                json_data.close()
                self._msg.i("Options overridden with config file.")
        except FileNotFoundError:
            self._msg.e("Config file not found! Using default config.")
        except json.decoder.JSONDecodeError as e:
            self._msg.e("Config syntax error: ")
            self._msg.e(e)
            exit(3)
    
    ## TODO: Descriptions
    def get(self, key, default = None):
        # Check if there has been a default value set
        if default is None:
            val = None
            try: # Try getting the values in loaded _config, otherwise try getting the default val in _def_config
                val = self._config.get(key, self._def_config[key])
            except KeyError as e:
                self._msg.e("IN Config.py:  %s" % e)
                val = self._def_config.get(key, None)
            return val
        else:   # Default val overridden by user
            return self._config.get(key, default)
    
    def _override(self, key, json):
        self._config[key] = json.get(key, self._def_config[key])
    
