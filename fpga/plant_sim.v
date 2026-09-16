module plant_simulator (
    input wire clk,          // 27MHz system clock
    input wire rst_n,        // Active-low reset
    input wire [7:0] pwm_in, // Control effort from ESP32 (0-255)
    output reg [15:0] enc_pos // Simulated encoder position output
);

    // Fixed-point scaling parameters for DC Motor Simulation
    // d_omega = (Kt/J * v) - (b/J * omega)
    reg [31:0] velocity = 0;
    reg [31:0] position = 0;
    
    // Prescaler to slow down simulation step to 1ms (27,000 cycles)
    reg [15:0] tick_counter = 0;
    parameter TICK_LIMIT = 27000; 

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            velocity <= 0;
            position <= 0;
            tick_counter <= 0;
            enc_pos <= 0;
        end else begin
            if (tick_counter >= TICK_LIMIT) begin
                tick_counter <= 0;
                
                // Simplified discrete motor dynamics update
                // velocity_next = velocity + (pwm_in * scaling) - (velocity >> damping_shift)
                velocity <= velocity + ({24'd0, pwm_in} - (velocity >> 3));
                
                // Integrate velocity to get position (encoder counts)
                position <= position + (velocity >> 8);
                enc_pos <= position[15:0];
            end else begin
                tick_counter <= tick_counter + 1;
            end
        end
    end

endmodule
