import SpecialMessages
import json
import argparse


class Config:

    # Required keys - Exit if not included in config file and arguments {Syntax = "key": (data_type, description, shorthand)}
    _required_keys = {
        "mqtt_base_topic": (str, "Mqtt base topic to respond to or send messages to", "t"),
        "mqtt_server": (str, "Mqtt server to connect to", "s"),
        "serial_port": (str, "Serial port to connect to", "p"),
    }

    # Optional keys - Return default value if not included in config file and arguments {Syntax = "key": (data_type, default_val, description, shorthand)}
    _optional_keys = {
        "mqtt_port": (int, 1883, "Mqtt server port", "P"),
        "mqtt_user": (str, None, "User for mqtt authentication", "u"),
        "mqtt_pass": (str, None, "Password for mqtt authentication (Requires mqtt_user)", "k"),
        "serial_baud": (int, 9600, "Serial baudrate", "b"),
        "log_level": (int, 3, "Verbosity [0 = Nothing, 1 = Error, 2 = Normal, 3 = Warning, 4 = Info, 5 = Debug]", "v")
    }

    _config = {}  # loaded configuration

    _msg = SpecialMessages.SpecialMessages(5)

    # Class initializer
    # Can supply multiple fallback paths
    def __init__(self, priority_path):

        # Load Arguments, Parse Config file location first
        argument_parser = argparse.ArgumentParser(description = "Irrigation manager")
        argument_parser.add_argument("-c", "--config", help = "Specify the configuration file location, Using this will make the required variables optional")

        config_file = argument_parser.parse_known_args()[0].config

        # Iterate through all the required keys (Required if --config is set)
        for key, value in self._required_keys.items():
            data_type, description, shorthand = value
            argument_parser.add_argument("-%s" % shorthand, "--%s" % key, help = description, type = data_type, required = (config_file is None))

        # Iterate through all the optional keys
        for key, value in self._optional_keys.items():
            data_type, default_val, description, shorthand = value
            argument_parser.add_argument("-%s" % shorthand, "--%s" % key, help = description, type = data_type, default = default_val)

        # Load config file, priority_path first
        return_code = self._load(priority_path)
        if return_code == 0:
            pass

        elif return_code == 1:
            # When overridden config not found, try parsing --config
            if config_file is not None:
                return_code = self._load(config_file)

                if return_code == 0:
                    pass

                else:
                    self._msg.e("Quitting!")
                    exit(10)

        elif return_code == 2:
            self._msg.e("Quitting!")
            exit(10)

        # Override local config
        arguments = vars(argument_parser.parse_args())

        def iteration(_key):
            if arguments[_key] is not None:
                self._config[_key] = arguments[_key]

        for key in self._required_keys.keys():
            iteration(key)

        for key in self._optional_keys.keys():
            iteration(key)


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

                # Check config for any required keys and load values, Will throw an error if a key is missing
                if not self._check_required_keys(d):
                    return 4

                # Now check for any optional keys
                self._check_optional_keys(d)

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
    def _check_required_keys(self, json_obj):
        # Iterate through all the required keys
        for key, value in self._required_keys.items():
            value, *_ = value
            try:
                # Check if the key is in the right format, otherwise return false
                if type(json_obj[key]) is value:
                    # Attempt to set get the key value, if the key doesn't exist, return false
                    self._config[key] = json_obj[key]
                else:
                    self._msg.e("Required key '%s' is not an '%s' type!" % (key, value))
                    break
            except KeyError:
                self._msg.e("Required key '%s' not found in config file!" % key)
                break

        # No errors found, Return true
        else:
            return True

        # An error has happened, return false
        return False

    # Check if any optional keys are present, override key if an value has been set
    def _check_optional_keys(self, json_obj):
        # Iterate through all the optional keys
        for key, value in self._optional_keys.items():
            type_val, default_val, *_ = value
            self._config[key] = default_val
            try:
                # Check if the key is in the right format, otherwise return false
                if type(json_obj[key]) is type_val:
                    # Attempt to set get the key value, if the key doesn't exist, return false
                    self._config[key] = json_obj[key]
                else:
                    self._msg.d("Optional key '%s' is not an '%s' type! Setting to the default value of '%s'" % (key, type_val, default_val))

            except KeyError:
                self._msg.d("Optional key '%s' not found in config file! Setting to the default value of '%s'" % (key, default_val))
