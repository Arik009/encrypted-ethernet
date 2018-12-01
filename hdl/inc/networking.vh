// measured in bytes/octets

localparam ETH_PREAMBLE_LEN = 8;
localparam ETH_CRC_LEN = 4;
localparam ETH_GAP_LEN = 12;
localparam ETH_MAC_LEN = 6;
localparam ETH_ETHERTYPE_LEN = 2;

localparam MAC_SEND = 48'hDEADBEEFCAFE;
localparam MAC_RECV = 48'hC0FFEEDAD101;

localparam ETHERTYPE_FGP = 16'hca11;
localparam ETHERTYPE_IP = 16'h0800;
localparam ETHERTYPE_ARP = 16'h0800;

localparam FGP_OFFSET_LEN = 1;
localparam FGP_DATA_LEN = 768;
localparam FGP_DATA_LEN_COLORS = FGP_DATA_LEN * BYTE_LEN / COLOR_LEN;
localparam FGP_LEN = FGP_OFFSET_LEN + FGP_DATA_LEN;
