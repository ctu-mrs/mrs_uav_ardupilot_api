Arducopter FW flash 101: soubor s bootloaderem a FW je moc velkej aby se vesel do pameti, musi se na to jit postupne:
1) flashnout FW s dfu-util: KakuteH7-bdshot_bl.bin
2) QGround FW update: arducopter.apj

Nastaveni message rates pres serial - umistit message-intervals textak do root dir SD karty.
Cislovani jde podle serial portu s nastavenym mavlink protokolem, e.g. chan0 -> serial0, chan1 -> serial2.
