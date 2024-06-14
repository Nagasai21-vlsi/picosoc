`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 16.04.2024 03:40:46
// Design Name: 
// Module Name: sevensegdisp
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module sevensegdisp(
  input [3:0] data,   // Input data (4 bits)
  output reg [6:0] seg // 7-segment display output (7 bits)
);

// 7-segment decoder logic to convert input data to display segments
always @*
  case (data)
    4'b0000: seg = 7'b0000001; // Display '0'
    4'b0001: seg = 7'b1001111; // Display '1'
    4'b0010: seg = 7'b0010010; // Display '2'
    4'b0011: seg = 7'b0000110; // Display '3'
    4'b0100: seg = 7'b1001100; // Display '4'
    4'b0101: seg = 7'b0100100; // Display '5'
    4'b0110: seg = 7'b0100000; // Display '6'
    4'b0111: seg = 7'b0001111; // Display '7'
    4'b1000: seg = 7'b0000000; // Display '8'
    4'b1001: seg = 7'b0000100; // Display '9'
    4'b1010: seg = 7'b0001000; // Display 'A'
    4'b1011: seg = 7'b1100000; // Display 'B'
    4'b1100: seg = 7'b0110001; // Display 'C'
    4'b1101: seg = 7'b1000010; // Display 'D'
    4'b1110: seg = 7'b0110000; // Display 'E'
    4'b1111: seg = 7'b0111000; // Display 'F'
    default: seg = 7'b1111111; // Turn off display if invalid data

  endcase

endmodule
