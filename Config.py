import SpecialMessages
import json


class Config:

    # Required keys - Exit if not included in config file (Syntax = "key": data_type)
    _required_keys = {
        "mqtt_base_topic": str,
        "mqtt_server": str,
        "mqtt_port": int,
        "serial_port": str,
        "serial_baud": int,
        "log_level": int
    }

    # Optional keys - Return default value if not included (Syntax = "key": [data_type, default_val})
    _optional_keys = {
        "mqtt_user": [str, None],
        "mqtt_pass": [str, None]
    }
    
    _config = {}  # loaded configuration
    
    _msg = SpecialMessages.SpecialMessages(5)

    # Class initializer
    # Can supply multiple fallback paths
    def __init__(self, *path):
        for file in path:
            if self._load(file) == 0:
                break
        else:
            self._msg.e("Quitting!")
            exit(10)

    # Get a configuration setting
    def get(self, key, default=None):
        # Check if there has been a default value set
        if default is None:
            val = None
            try:  # Try getting the values in loaded _config, otherwise return none
                val = self._config.get(key, None)
            except KeyError as e:
                self._msg.e("IN Config.py:  %s" % e)
            return val
        else:   # Default val overridden by user
            return self._config.get(key, default)
    
    # Try to load a config file by path, returns 0 on success
    def _load(self, path):
        try:
            with open(path) as json_data:
                d = json.load(json_data)

                # Check config for any required keys and load values
                if not self._check_config(d):
                    return 4

                # Close the file properly
                json_data.close()

                self._msg.i("Config file '%s' loaded." % path)
                return 0

        except FileNotFoundError:
            self._msg.e("Config file '%s' not found!" % path)
            return 1

        except json.decoder.JSONDecodeError as e:
            self._msg.e("Config syntax error: ")
            self._msg.e(e)
            return 2

    # Check if the required keys are present, Returns boolean
    def _check_config(self, json_obj):
        # Iterate through all the required keys
        for i in self._required_keys:
            try:
                # Check if the key is in the right format, otherwise return false
                if type(json_obj[i]) is self._required_keys[i]:
                    # Attempt to set get the key value, if the key doesn't exist, return false
                    self._config[i] = json_obj[i]
                else:
                    self._msg.e("Required key '%s' is not an '%s' type!" % (i, self._required_keys[i]))
                    break
            except KeyError:
                self._msg.e("Required key '%s' not found in config file!" % i)
                break

        # No errors found, Return true
        else:
            return True

        # An error has happened, return false
        return False
