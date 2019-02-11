package me.drton.jmavsim;

import me.drton.jmavlib.mavlink.MAVLinkMessage;

public interface Peripherial  {
    void filterMessage(MAVLinkMessage msg);
}
