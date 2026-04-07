`timescale 1ns / 1ps

module spi_master_tb;

    reg         clk = 0;
    reg         rst_n = 0;
    reg         active = 0;
    reg  [3:0]  addr = 0;
    reg  [31:0] wdata = 0;
    reg  [3:0]  wstrb = 0;
    wire [31:0] rdata;
    wire        ready;
    wire        spi_sck, spi_mosi, spi_cs;
    reg         spi_miso = 1;

    // 25 MHz clock
    always #20 clk = ~clk;

    spi_master #(
        .DIV_SLOW(3),   // Speed up for simulation: CLK/8
        .DIV_FAST(0)    // CLK/2
    ) uut (
        .clk(clk), .rst_n(rst_n),
        .active(active), .addr(addr),
        .wdata(wdata), .wstrb(wstrb),
        .rdata(rdata), .ready(ready),
        .spi_sck(spi_sck), .spi_mosi(spi_mosi),
        .spi_miso(spi_miso), .spi_cs(spi_cs)
    );

    // SPI loopback: MISO mirrors MOSI delayed by half SCK period
    // (simulates an SD card echoing back)
    always @(posedge spi_sck) spi_miso <= spi_mosi;

    // Tasks for register access
    task write_reg(input [3:0] a, input [31:0] d);
        begin
            @(posedge clk);
            active <= 1; addr <= a; wdata <= d; wstrb <= 4'hF;
            @(posedge clk);
            while (!ready) @(posedge clk);
            active <= 0; wstrb <= 0;
            @(posedge clk);
        end
    endtask

    task read_reg(input [3:0] a);
        begin
            @(posedge clk);
            active <= 1; addr <= a; wstrb <= 0;
            @(posedge clk);
            while (!ready) @(posedge clk);
            active <= 0;
            @(posedge clk);
        end
    endtask

    initial begin
        $dumpfile("spi_master_tb.vcd");
        $dumpvars(0, spi_master_tb);

        // Reset
        #100;
        rst_n <= 1;
        #100;

        $display("=== SPI Master Testbench ===");

        // 1. Check initial state: CS high, SCK low
        if (spi_cs !== 1'b1) $display("FAIL: CS should be high after reset");
        else $display("PASS: CS high after reset");

        if (spi_sck !== 1'b0) $display("FAIL: SCK should be low after reset");
        else $display("PASS: SCK low after reset");

        // 2. Set CS low (slow mode, cs_val=0)
        write_reg(4'h8, 32'h04);  // CTRL: slow_mode=1, cs_val=0
        #40;
        if (spi_cs !== 1'b0) $display("FAIL: CS should be low after CTRL write");
        else $display("PASS: CS asserted");

        // 3. Send byte 0xA5 in slow mode
        $display("Sending 0xA5...");
        write_reg(4'h0, 32'h000000A5);  // DATA write starts transfer

        // Wait for transfer to complete
        #10000;

        // 4. Read STATUS — should not be busy
        read_reg(4'h4);
        $display("STATUS after xfer: busy=%0d done=%0d", rdata[0], rdata[1]);

        // 5. Read RX byte — with loopback, should get shifted version of TX
        read_reg(4'h0);
        $display("RX byte: 0x%02X", rdata[7:0]);

        // 6. Switch to fast mode, send 0xFF
        write_reg(4'h8, 32'h00);  // CTRL: fast_mode, cs_val=0
        #40;

        $display("Sending 0xFF (fast mode)...");
        write_reg(4'h0, 32'h000000FF);
        #2000;

        read_reg(4'h0);
        $display("RX byte (fast): 0x%02X", rdata[7:0]);

        // 7. Deassert CS
        write_reg(4'h8, 32'h01);  // cs_val=1
        #40;
        if (spi_cs !== 1'b1) $display("FAIL: CS should be high");
        else $display("PASS: CS deasserted");

        $display("=== Done ===");
        #200;
        $finish;
    end

endmodule
