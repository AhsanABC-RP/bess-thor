# 1970-01-02 00:59:34 by RouterOS 7.16.1
# software id = I01W-KNHF
#
# model = CRS518-16XS-2XQ
# serial number = HJ40AFS9PZV
/interface bridge
# MTU > L2MTU
add admin-mac=F4:1E:57:C9:AF:8B auto-mac=no comment=defconf mtu=9000 name=\
    bridgeLocal protocol-mode=none
add name=bridgeWAN
/interface ethernet
set [ find default-name=sfp28-1 ] l2mtu=9216
set [ find default-name=sfp28-2 ] l2mtu=9216
set [ find default-name=sfp28-6 ] l2mtu=9216
set [ find default-name=sfp28-7 ] sfp-shutdown-temperature=130C
set [ find default-name=sfp28-8 ] l2mtu=9216 mtu=1584 \
    sfp-shutdown-temperature=130C
set [ find default-name=sfp28-9 ] auto-negotiation=no fec-mode=off speed=\
    10G-baseCR
set [ find default-name=sfp28-11 ] auto-negotiation=no fec-mode=off speed=\
    10G-baseCR
set [ find default-name=sfp28-13 ] auto-negotiation=no fec-mode=off speed=\
    10G-baseCR
set [ find default-name=sfp28-15 ] auto-negotiation=no fec-mode=off speed=\
    10G-baseCR
set [ find default-name=sfp28-16 ] auto-negotiation=no fec-mode=off l2mtu=\
    9216 mtu=1584 speed=10G-baseCR
/interface lte
set [ find default-name=lte1 ] sms-protocol=auto sms-read=no
/port
set 0 baud-rate=115200 name=serial0
/interface bridge port
add bridge=bridgeLocal comment=defconf interface=sfp28-16
add bridge=bridgeLocal interface=sfp28-4
add bridge=bridgeLocal interface=sfp28-5
add bridge=bridgeLocal interface=sfp28-6
add bridge=bridgeLocal interface=sfp28-3
add bridge=bridgeLocal interface=sfp28-1
add bridge=bridgeLocal interface=sfp28-2
add bridge=bridgeLocal interface=sfp28-9
add bridge=bridgeLocal interface=sfp28-11
add bridge=bridgeLocal interface=sfp28-13
add bridge=bridgeLocal interface=sfp28-15
add bridge=bridgeLocal interface=sfp28-7
add bridge=bridgeLocal interface=sfp28-8
add bridge=bridgeLocal interface=sfp28-10
add bridge=bridgeLocal interface=sfp28-14
add bridge=bridgeWAN interface=sfp28-12
/ip address
add address=169.254.100.254/16 interface=bridgeLocal network=169.254.0.0
add address=192.168.2.200/24 interface=bridgeLocal network=192.168.2.0
/ip dhcp-client
add comment=defconf interface=bridgeLocal
add comment="RUTX50 5G WAN" default-route-distance=5 interface=bridgeWAN \
    use-peer-dns=no use-peer-ntp=no
/ip firewall nat
add action=masquerade chain=srcnat comment="masq to RUTX50 WAN" \
    out-interface=bridgeWAN
/port remote-access
add port=serial0 protocol=raw tcp-port=9999
/system note
set show-at-login=no
/system routerboard settings
set enter-setup-on=delete-key
/tool sniffer
set filter-direction=rx filter-interface=sfp28-12 memory-limit=10KiB
