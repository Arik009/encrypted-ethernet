`include "params.vh"

// measured in bytes/octets

localparam ETH_PREAMBLE_LEN = 8;
localparam ETH_CRC_LEN = 4;
localparam ETH_GAP_LEN = 12;
localparam ETH_MAC_LEN = 6;
localparam ETH_ETHERTYPE_LEN = 2;

localparam ETHERTYPE_FGP = 16'hca11;
localparam ETHERTYPE_FFCP = 16'hca12;

localparam FGP_OFFSET_LEN = 1;
localparam FGP_DATA_LEN = 768;
localparam FGP_DATA_LEN_COLORS = FGP_DATA_LEN * BYTE_LEN / COLOR_LEN;
localparam FGP_LEN = FGP_OFFSET_LEN + FGP_DATA_LEN;

// in bits
localparam FFCP_TYPE_LEN = 2;
localparam FFCP_INDEX_LEN = 6;
// back to bytes
localparam FFCP_METADATA_LEN = 1;
localparam FFCP_DATA_LEN = FGP_LEN;
localparam FFCP_LEN = FFCP_METADATA_LEN + FFCP_DATA_LEN;

localparam FFCP_BUFFER_LEN = 2**FFCP_INDEX_LEN;
localparam FFCP_WINDOW_LEN = 4;

localparam PB_PARTITION_LEN = 2**clog2(FFCP_LEN);
localparam PB_QUEUE_LEN = PACKET_BUFFER_SIZE / PB_PARTITION_LEN;
localparam PB_QUEUE_ALMOST_FULL_THRES = PB_QUEUE_LEN * 7 / 8;
