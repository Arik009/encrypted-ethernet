localparam BYTE_LEN_LOG2 = 3;
localparam BYTE_LEN = 2 ** BYTE_LEN_LOG2;
localparam PACKET_BUFFER_SIZE_LOG2 = 12;
localparam PACKET_BUFFER_SIZE = 2 ** PACKET_BUFFER_SIZE_LOG2;
// taken from ip summary
localparam PACKET_BUFFER_READ_LATENCY = 2;
