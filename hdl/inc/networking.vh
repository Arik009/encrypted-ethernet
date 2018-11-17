// measured in bytes/octets

localparam ETH_PREAMBLE_LEN = 8;
localparam ETH_CRC_LEN = 4;
localparam ETH_GAP_LEN = 12;
localparam ETH_MAC_LEN = 6;
localparam ETH_ETHERTYPE_LEN = 2;

localparam FGP_OFFSET_LEN = 1;
localparam FGP_PADDING_LEN = 127;
localparam FGP_DATA_LEN = 768;
localparam FGP_DATA_LEN_COLORS = FGP_DATA_LEN * BYTE_LEN / COLOR_LEN;
