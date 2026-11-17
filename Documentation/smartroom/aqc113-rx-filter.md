# DRAFT: AQC113 receive filter (for Lucas, do not send as-is)

Target: omacom/linux-aurora only. Not for AsahiLinux.

Branch: `smartroom/aqc113-rx-filter`

Upstream commits this is based on, from linux-asahi (Janne Grunau, 2026-10-26):

- `d96f2a368faa` net: atlantic: AQC113: Fix hw_packet_filter_set for l2 filter base index > 0
- `ffe05aa84b17` net: atlantic: AQC113: Limit multicast filters to available slots

Those two are not in omacom/linux-aurora (nor in `smartroom/main`). linux-asahi
and linux-aurora share the June 2026 AQC113 filter work (`6c1ca112a6e4`) and
then diverge. The October commits are the missing half.

One predicate in `d96f2a368faa` is adjusted here. That commit starts the L2
walk at `l2_filters_base_index + 1` but still enables slot `i` when
`i <= mc_list_count`. After the walk moves, `i` is an absolute slot (10 and
up on these Macs) and `mc_list_count` is still a count, so the test is never
true and every multicast slot is cleared again. The enable test is now
`(i - base_location) < mc_list_count`. With base 1 that is the same set of
slots the A1 path enabled before.

## Why this is the bug

On the M2 Max Studio the AQC113 firmware (ATL2FW 103001b / 1.3.27) reports
`l2_filters_base_index` 9. `hw_atl2_hw_mac_addr_set()` writes the station MAC
at that slot and tags it for the action resolver
(`HW_ATL2_RPF_TAG_BASE_UC`). The resolver entry is already installed by the
June init path. Broadcast has its own enable bit.

`aq_nic_set_multicast_list()` then calls `hw_packet_filter_set()`. That wraps
`hw_atl_b0_hw_packet_filter_set()`, which walks every L2 slot from
`HW_ATL_B0_MAC_MIN` (1) and leaves a slot enabled only when
`i <= mc_list_count`. Slot 9 is not in that range, so the station filter is
turned off. Multicast addresses were just programmed at slots 10 and up, and
the same walk turns those off too.

What still works: transmit (the netdev MAC is correct), and broadcast ARP
(separate broadcast filter). What does not: unicast to `a4:fc:14:2d:d4:fc`,
and IPv6 multicast, including router advertisements. Promiscuous mode sets
the L2 promisc bit and skips the table, which is why
`802-3-ethernet.accept-all-mac-addresses=yes` hides it.

The September 2026 Asahi ALARM kernel got DHCP on this port. Current
linux-asahi has the October filter fixes; the package we ship
(`7.2.11-14-1-ARCH`) does not.

## Suggested PR text

Subject: net: atlantic: keep the AQC113 station filter when the firmware base index is not 0

On Apple silicon desktops the AQC113 (atlantic, hw_atl2) brings the link up
and transmits, and broadcast ARP is received, but unicast to the station MAC
and IPv6 multicast are dropped unless the interface is promiscuous. The MAC
in the device tree is correct (`addr_assign_type` 0).

`hw_atl2_hw_mac_addr_set()` programs that MAC at `l2_filters_base_index`
(9 on ATL2FW 1.3.27) and the action resolver already delivers that tag.
The next `hw_packet_filter_set()` calls the A1 helper, which enables and
disables L2 slots from index 1 and so clears the station slot and the
multicast slots above it.

Start that walk at the first multicast slot, compare against the multicast
count rather than the absolute index, and refuse a multicast list that does
not fit in the firmware window (`l2_filter_count`, one slot reserved for the
station address).

Fixes: 6c1ca112a6e4 ("net: atlantic: fix AQC113 HW init: ART, L2 filter slot, MAC address")

Related: omacom/omarchy-mac-pkgs#137

## Tested

On the M2 Max Studio (j475c, t6021), linux-aurora `7.2.11.aurora14-1.sr1`
(`7.2.11-14-1.sr1-ARCH`), wired link with promiscuous mode off
(`promiscuity` 0):

- DHCP lease arrived in 0.2 s.
- IPv6 SLAAC completed.
- Unicast ping to the router and to the Mini succeeded.

The filter change above is the patch to send. No further code change.

## Test plan (Claude, on the Studio)

Promiscuous mode off first:

```
nmcli connection modify 'Wired connection 1' 802-3-ethernet.accept-all-mac-addresses -1
nmcli connection up 'Wired connection 1' ifname end0
ip -d link show end0
```

`ip link` must not show PROMISC. `promiscuity` must be 0.

1. Unicast ping. From another LAN host, ping the Studio's DHCP address.
   On the Studio, `ping -I end0 -c 3 <gateway>` must get replies.
2. IPv6 router advertisement. `tcpdump -p -ni end0 -c 2 icmp6 and ip6[40] == 134`
   must show an RA within a minute. The `-p` flag is required: tcpdump
   turns promiscuous mode on unless it is set, which hides this bug.
3. DHCP. `nmcli connection down` / `up` the wired profile, or
   `dhclient -v end0` if NetworkManager is stopped. A lease must arrive
   while PROMISC is off.
4. Broadcast still works: `arping -I end0 <gateway>` from the Studio.

If it fails, note `dmesg | grep -i atlantic` and whether `ip -d link`
shows PROMISC. Wi-Fi is the fallback path. Restore the shipping workaround
only when the test fails, so the wired link is usable again:

```
nmcli connection modify 'Wired connection 1' 802-3-ethernet.accept-all-mac-addresses 1
nmcli connection up 'Wired connection 1' ifname end0
```
