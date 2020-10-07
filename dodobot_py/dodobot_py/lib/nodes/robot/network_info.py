import re
import subprocess


class NetworkInfo:
    def __init__(self, interfaces: list):
        self.interfaces = interfaces
        self.info = {k: {
            "name": k,
            "ip": "XX.XX.XX.XX",
            "nmask": "XX.XX.XX.XX",
            "bcast": "XX.XX.XX.XX",
            "error": ""
        } for k in self.interfaces}
        self.search_regex = r"inet(.*)netmask(.*)broadcast(.*)"
        self.error_regex = r"error fetching interface information:"

    def update(self, ):
        for interface_name in self.interfaces:
            self.get_info(interface_name)

    def get_info(self, interface_name):
        interface_info = self.info[interface_name]
        interface_info["name"] = interface_name

        result = subprocess.run(["ifconfig", interface_name], stdout=subprocess.PIPE)
        output = result.stdout.decode()

        match = re.search(self.error_regex, output)
        if match:
            interface_info["error"] = output
        else:
            match = re.search(self.search_regex, output)

            if match is not None:
                interface_info["ip"] = match.group(1)
                interface_info["nmask"] = match.group(2)
                interface_info["bcast"] = match.group(3)
                interface_info["error"] = ""
            else:
                interface_info["error"] = "Interface not connected"
