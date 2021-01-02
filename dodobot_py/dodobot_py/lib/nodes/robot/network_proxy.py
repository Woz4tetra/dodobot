import re
# import textwrap
import subprocess


class NetworkProxy:
    def __init__(self):
        self.connection_cmd = "nmcli -p -m multiline -f common device show".split(" ")
        self.list_cmd = "nmcli -p -m multiline -f common device".split(" ")
        self.hostname_cmd = ["hostname"]
        self.wifi_state_command = "nmcli radio wifi".split(" ")
        self.wifi_on_command = "nmcli radio wifi on".split(" ")
        self.wifi_off_command = "nmcli radio wifi off".split(" ")

        self.device_regex = r"GENERAL\.DEVICE:(.*)"  # interface name
        self.device_state_regex = r"GENERAL\.STATE:.*\((.*)\)"  # connected or not
        self.connection_regex = r"GENERAL\.CONNECTION:(.*)"  # wifi SSID
        self.ip_address = r"IP4\.ADDRESS.*:(.*)"  # ip address

        self.device_regex = r"DEVICE:.* (.*)"  # device name
        self.device_state_regex_list = r"STATE:.* (.*)"  # connection state

        # self.network_info_wrapper = textwrap.TextWrapper(width=17, break_long_words=True, replace_whitespace=False)

    def list_devices(self):
        result = subprocess.run(self.list_cmd, stdout=subprocess.PIPE)
        output = result.stdout.decode()

        devices = []
        matches = re.finditer(self.device_regex, output, re.MULTILINE)
        for match in matches:
            device_name = match.group(1)
            devices.append(device_name)

        conn_states = []
        matches = re.finditer(self.device_state_regex_list, output, re.MULTILINE)
        for index, match in enumerate(matches):
            state = match.group(1)
            conn_states.append(state)

        assert len(conn_states) == len(devices), f"{len(conn_states)} != {len(devices)}"

        report = {}
        for index in range(len(devices)):
            report[devices[index]] = conn_states[index]
        return report

    def get_interface_info(self, interface_name):
        result = subprocess.run(self.connection_cmd + [str(interface_name)], stdout=subprocess.PIPE)
        output = result.stdout.decode()

        device_name = "--"
        device_state = "--"
        connection_state = "--"
        ip_address = "xx.xx.xx.xx"

        match = re.search(self.device_regex, output)
        if match:
            device_name = match.group(1).strip()
        else:
            print("Device name doesn't match interface name! %s != %s" % (device_name, interface_name))

        match = re.search(self.device_state_regex, output)
        if match:
            device_state = match.group(1).strip()

        match = re.search(self.connection_regex, output)
        if match:
            connection_state = match.group(1).strip()

        match = re.search(self.ip_address, output)
        if match:
            ip_address = match.group(1).strip()

        report = f"{device_name} {device_state}\n{connection_state}\n{ip_address}"
        return report

    def get_hostname(self):
        result = subprocess.run(self.hostname_cmd, stdout=subprocess.PIPE)
        output = result.stdout.decode()
        return output.strip()

    def generate_report(self, devices):
        report = self.get_hostname() + ".local\n"
        for device, state in devices.items():
            if state == "unmanaged":
                continue
            device_report = self.get_interface_info(device)
            # report += "\n".join(self.network_info_wrapper.wrap(device_report))
            report += device_report
            report += "\n\n"
        return report.strip()

    def get_report(self):
        devices = self.list_devices()
        is_connected = False
        for state in devices.values():
            if state != "unmanaged":
                is_connected = True
                break

        if is_connected:
            report = self.generate_report(devices)
        else:
            report = "Disconnected"
        return report, devices

    def get_radio_state(self):
        result = subprocess.run(self.wifi_state_command, stdout=subprocess.PIPE)
        output = result.stdout.decode()
        return "enabled" in output

    def set_radio_state(self, state):
        if state:
            cmd = self.wifi_on_command
        else:
            cmd = self.wifi_off_command
        result = subprocess.run(cmd, stdout=subprocess.PIPE)
        output = result.stdout.decode()
        return output

if __name__ == '__main__':
    def test():
        proxy = NetworkProxy()
        devices = proxy.list_devices()
        print(proxy.generate_report(devices))

    test()
