-- #################################################################################################
-- # << The NEORV32 RISC-V Processor - Top Entity >>                                               #
-- # ********************************************************************************************* #
-- # Check out the processor's online documentation for more information:                          #
-- #  HQ:         https://github.com/stnolting/neorv32                                             #
-- #  Data Sheet: https://stnolting.github.io/neorv32                                              #
-- #  User Guide: https://stnolting.github.io/neorv32/ug                                           #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # Copyright (c) 2023, Stephan Nolting. All rights reserved.                                     #
-- #                                                                                               #
-- # Redistribution and use in source and binary forms, with or without modification, are          #
-- # permitted provided that the following conditions are met:                                     #
-- #                                                                                               #
-- # 1. Redistributions of source code must retain the above copyright notice, this list of        #
-- #    conditions and the following disclaimer.                                                   #
-- #                                                                                               #
-- # 2. Redistributions in binary form must reproduce the above copyright notice, this list of     #
-- #    conditions and the following disclaimer in the documentation and/or other materials        #
-- #    provided with the distribution.                                                            #
-- #                                                                                               #
-- # 3. Neither the name of the copyright holder nor the names of its contributors may be used to  #
-- #    endorse or promote products derived from this software without specific prior written      #
-- #    permission.                                                                                #
-- #                                                                                               #
-- # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS   #
-- # OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF               #
-- # MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE    #
-- # COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,     #
-- # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE #
-- # GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED    #
-- # AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING     #
-- # NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED  #
-- # OF THE POSSIBILITY OF SUCH DAMAGE.                                                            #
-- # ********************************************************************************************* #
-- # The NEORV32 Processor - https://github.com/stnolting/neorv32              (c) Stephan Nolting #
-- #################################################################################################

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_top is
    generic (
        -- General --
        CLOCK_FREQUENCY : natural; -- clock frequency of clk_i in Hz

        -- Physical Memory Protection (PMP) --
        MEM_INT_IMEM_SIZE : natural := 32 * 1024; -- size of processor-internal instruction memory in bytes

        -- Internal Data memory (DMEM) --
        MEM_INT_DMEM_SIZE : natural := 32 * 1024; -- size of processor-internal data memory in bytes

        -- External Interrupts Controller (XIRQ) --
        XIRQ_NUM_CH : natural := 1; -- number of external IRQ channels (0..32)
        XIRQ_TRIGGER_TYPE : std_ulogic_vector(31 downto 0) := x"ffffffff"; -- trigger type: 0=level, 1=edge
        XIRQ_TRIGGER_POLARITY : std_ulogic_vector(31 downto 0) := x"ffffffff"; -- trigger polarity: 0=low-level/falling-edge, 1=high-level/rising-edge

        -- Processor peripherals --
        IO_GPIO_NUM : natural := 0 -- number of GPIO input/output pairs (0..64)

    );
    port (
        -- Global control --
        clk_i : in std_ulogic; -- global clock, rising edge
        rstn_i : in std_ulogic; -- global reset, low-active, async
        -- XIP (execute in place via SPI) signals (available if IO_XIP_EN = true) --
        xip_csn_o : out std_ulogic; -- chip-select, low-active
        xip_clk_o : out std_ulogic; -- serial clock
        xip_dat_i : in std_ulogic := 'L'; -- device data input
        xip_dat_o : out std_ulogic; -- controller data output
        -- GPIO (available if IO_GPIO_NUM > 0) --
        gpio_o : out std_ulogic_vector(63 downto 0); -- parallel output
        gpio_i : in std_ulogic_vector(63 downto 0) := (others => 'U'); -- parallel input
        -- primary UART0 (available if IO_UART0_EN = true) --
        uart0_txd_o : out std_ulogic; -- UART0 send data
        uart0_rxd_i : in std_ulogic := 'U'; -- UART0 receive data
        -- External platform interrupts (available if XIRQ_NUM_CH > 0) --
        xirq_i : in std_ulogic_vector(31 downto 0) := (others => 'L') -- IRQ channels
    );
end neorv32_top;

architecture neorv32_top_rtl of neorv32_top is

    -- CPU boot configuration --
    constant cpu_boot_addr_c : std_ulogic_vector(31 downto 0) := cond_sel_stdulogicvector_f(true, boot_rom_base_c, ispace_base_c);

    -- reset generator --
    signal rstn_ext_sreg, rstn_int_sreg : std_ulogic_vector(3 downto 0);
    signal rstn_ext, rstn_int, rstn_wdt : std_ulogic;

    -- clock generator --
    signal clk_div : std_ulogic_vector(11 downto 0);
    signal clk_div_ff : std_ulogic_vector(11 downto 0);
    signal clk_gen : std_ulogic_vector(07 downto 0);
    signal clk_gen_en : std_ulogic_vector(10 downto 0);
    signal clk_gen_en_ff : std_ulogic;
    --
    constant wdt_cg_en : std_ulogic := '0';
    signal uart0_cg_en : std_ulogic;
    constant uart1_cg_en : std_ulogic := '0';
    signal spi_cg_en : std_ulogic;
    constant twi_cg_en : std_ulogic := '0';
    constant pwm_cg_en : std_ulogic := '0';
    constant cfs_cg_en : std_ulogic := '0';
    constant neoled_cg_en : std_ulogic := '0';
    constant gptmr_cg_en : std_ulogic := '0';
    constant xip_cg_en : std_ulogic := '0';
    constant onewire_cg_en : std_ulogic := '0';

    -- CPU status --
    signal cpu_debug : std_ulogic; -- cpu is in debug mode
    signal cpu_sleep : std_ulogic; -- cpu is in sleep mode

    -- debug core interface (DCI) --
    constant dci_ndmrstn_c : std_ulogic := '1';
    constant dci_halt_req_c : std_ulogic := '0';

    -- uart0
    signal uart0_rts_o : std_ulogic;
    constant uart0_cts_i : std_ulogic := '0';

    -- machine interrupts
    constant mtime_irq_i : std_ulogic := '0';
    constant msw_irq_i : std_ulogic := '0';
    constant mext_irq_i : std_ulogic := '0';

    -- internal bus system --
    type device_ids_t is (DEV_BUSKEEPER, DEV_IMEM, DEV_DMEM, DEV_BOOTROM, DEV_GPIO, DEV_UART0,
        DEV_SYSINFO, DEV_XIRQ, DEV_XIP_CT, DEV_XIP_ACC, DEV_SPI);

    -- core complex --
    signal cpu_i_req : inst_req_t;
    signal cpu_d_req : bus_req_t; -- CPU core
    signal cpu_i_rsp, cpu_d_rsp : bus_rsp_t; -- CPU core
    signal core_req : bus_req_t; -- core complex (CPU + caches)
    signal core_rsp : bus_rsp_t; -- core complex (CPU + caches)

    -- SoC bus --
    type response_bus_t is array (device_ids_t) of bus_rsp_t;
    signal soc_req : bus_req_t; -- SoC request bus
    signal soc_rsp : bus_rsp_t; -- SoC response bus
    signal io_req : bus_req_t; -- request bus for internal IO/Peripheral devices only
    signal rsp_bus : response_bus_t; -- global response bus
    signal bus_error : std_ulogic; -- global bus error

    -- IRQs --
    signal fast_irq : std_ulogic_vector(15 downto 0);
    constant mtime_irq : std_ulogic := '0';
    constant wdt_irq : std_ulogic := '0';
    signal uart0_rx_irq : std_ulogic;
    signal uart0_tx_irq : std_ulogic;
    constant uart1_rx_irq : std_ulogic := '0';
    constant uart1_tx_irq : std_ulogic := '0';
    signal spi_irq : std_ulogic;
    constant sdi_irq : std_ulogic := '0';
    constant twi_irq : std_ulogic := '0';
    constant cfs_irq : std_ulogic := '0';
    constant neoled_irq : std_ulogic := '0';
    signal xirq_irq : std_ulogic;
    constant gptmr_irq : std_ulogic := '0';
    constant onewire_irq : std_ulogic := '0';
    constant dma_irq : std_ulogic := '0';
    constant trng_irq : std_ulogic := '0';

    -- misc --
    signal io_acc : std_ulogic;
    constant ext_timeout : std_ulogic := '0';
    constant ext_access : std_ulogic := '0';
    signal spi_csn : std_ulogic_vector(7 downto 0);
    signal xip_access : std_ulogic;
    signal xip_enable : std_ulogic;
    signal xip_page : std_ulogic_vector(3 downto 0);

begin

    -- ****************************************************************************************************************************
    -- Clock and Reset
    -- ****************************************************************************************************************************

    -- Reset Generator ------------------------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    reset_generator : process (rstn_i, clk_i)
    begin
        if (rstn_i = '0') then
            rstn_ext_sreg <= (others => '0');
            rstn_int_sreg <= (others => '0');
            rstn_ext <= '0';
            rstn_int <= '0';
        elsif falling_edge(clk_i) then -- inverted clock to release reset _before_ all FFs trigger (rising edge)
            -- external reset --
            rstn_ext_sreg <= rstn_ext_sreg(rstn_ext_sreg'left - 1 downto 0) & '1'; -- active for at least <rstn_ext_sreg'size> clock cycles
            -- internal reset --
            if (rstn_wdt = '0') or (dci_ndmrstn_c = '0') then -- sync reset sources
                rstn_int_sreg <= (others => '0');
            else
                rstn_int_sreg <= rstn_int_sreg(rstn_int_sreg'left - 1 downto 0) & '1'; -- active for at least <rstn_int_sreg'size> clock cycles
            end if;
            -- reset nets --
            rstn_ext <= and_reduce_f(rstn_ext_sreg); -- external reset (via reset pin)
            rstn_int <= and_reduce_f(rstn_int_sreg); -- internal reset (via reset pin, WDT or OCD)
        end if;
    end process reset_generator;

    -- Clock Generator ------------------------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    clock_generator : process (rstn_int, clk_i)
    begin
        if (rstn_int = '0') then
            clk_gen_en_ff <= '0';
            clk_div <= (others => '0');
            clk_div_ff <= (others => '0');
        elsif rising_edge(clk_i) then
            clk_gen_en_ff <= or_reduce_f(clk_gen_en);
            if (clk_gen_en_ff = '1') then
                clk_div <= std_ulogic_vector(unsigned(clk_div) + 1);
            else -- reset if disabled
                clk_div <= (others => '0');
            end if;
            clk_div_ff <= clk_div;
        end if;
    end process clock_generator;

    -- clock enables: rising edge detectors --
    clk_gen(clk_div2_c) <= clk_div(0) and (not clk_div_ff(0)); -- CLK/2
    clk_gen(clk_div4_c) <= clk_div(1) and (not clk_div_ff(1)); -- CLK/4
    clk_gen(clk_div8_c) <= clk_div(2) and (not clk_div_ff(2)); -- CLK/8
    clk_gen(clk_div64_c) <= clk_div(5) and (not clk_div_ff(5)); -- CLK/64
    clk_gen(clk_div128_c) <= clk_div(6) and (not clk_div_ff(6)); -- CLK/128
    clk_gen(clk_div1024_c) <= clk_div(9) and (not clk_div_ff(9)); -- CLK/1024
    clk_gen(clk_div2048_c) <= clk_div(10) and (not clk_div_ff(10)); -- CLK/2048
    clk_gen(clk_div4096_c) <= clk_div(11) and (not clk_div_ff(11)); -- CLK/4096

    -- fresh clocks anyone? --
    clk_gen_en(0) <= wdt_cg_en;
    clk_gen_en(1) <= uart0_cg_en;
    clk_gen_en(2) <= uart1_cg_en;
    clk_gen_en(3) <= spi_cg_en;
    clk_gen_en(4) <= twi_cg_en;
    clk_gen_en(5) <= pwm_cg_en;
    clk_gen_en(6) <= cfs_cg_en;
    clk_gen_en(7) <= neoled_cg_en;
    clk_gen_en(8) <= gptmr_cg_en;
    clk_gen_en(9) <= xip_cg_en;
    clk_gen_en(10) <= onewire_cg_en;

    -- ****************************************************************************************************************************
    -- Core Complex
    -- ****************************************************************************************************************************

    -- CPU Core -------------------------------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_cpu_inst : entity neorv32.neorv32_cpu
        generic map(
            -- General --
            HART_ID => x"00000000",
            VENDOR_ID => x"00000000",
            CPU_BOOT_ADDR => cpu_boot_addr_c,
            CPU_DEBUG_PARK_ADDR => dm_park_entry_c,
            CPU_DEBUG_EXC_ADDR => dm_exc_entry_c,
            -- RISC-V CPU Extensions --
            CPU_EXTENSION_RISCV_B => false,
            CPU_EXTENSION_RISCV_C => true,
            CPU_EXTENSION_RISCV_E => false,
            CPU_EXTENSION_RISCV_M => true,
            CPU_EXTENSION_RISCV_U => false,
            CPU_EXTENSION_RISCV_Zfinx => false,
            CPU_EXTENSION_RISCV_Zicntr => true,
            CPU_EXTENSION_RISCV_Zicond => false,
            CPU_EXTENSION_RISCV_Zihpm => false,
            CPU_EXTENSION_RISCV_Zifencei => false,
            CPU_EXTENSION_RISCV_Zmmul => false,
            CPU_EXTENSION_RISCV_Zxcfu => false,
            CPU_EXTENSION_RISCV_Sdext => false,
            CPU_EXTENSION_RISCV_Sdtrig => false,
            -- Extension Options --
            FAST_MUL_EN => true,
            FAST_SHIFT_EN => true,
            CPU_IPB_ENTRIES => 2,
            -- Hardware Performance Monitors (HPM) --
            HPM_NUM_CNTS => 0,
            HPM_CNT_WIDTH => 0
        )
        port map(
            -- global control --
            clk_i => clk_i,
            rstn_i => rstn_int,
            sleep_o => cpu_sleep,
            debug_o => cpu_debug,
            -- interrupts --
            msi_i => msw_irq_i,
            mei_i => mext_irq_i,
            mti_i => mtime_irq,
            firq_i => fast_irq,
            dbi_i => dci_halt_req_c,
            -- instruction bus interface --
            ibus_req_o => cpu_i_req,
            ibus_rsp_i => cpu_i_rsp,
            -- data bus interface --
            dbus_req_o => cpu_d_req,
            dbus_rsp_i => cpu_d_rsp
        );

    -- fast interrupt requests (FIRQs) --
    fast_irq(00) <= wdt_irq; -- highest priority
    fast_irq(01) <= cfs_irq;
    fast_irq(02) <= uart0_rx_irq;
    fast_irq(03) <= uart0_tx_irq;
    fast_irq(04) <= uart1_rx_irq;
    fast_irq(05) <= uart1_tx_irq;
    fast_irq(06) <= spi_irq;
    fast_irq(07) <= twi_irq;
    fast_irq(08) <= xirq_irq;
    fast_irq(09) <= neoled_irq;
    fast_irq(10) <= dma_irq;
    fast_irq(11) <= sdi_irq;
    fast_irq(12) <= gptmr_irq;
    fast_irq(13) <= onewire_irq;
    fast_irq(14) <= '0';
    fast_irq(15) <= trng_irq; -- lowest priority

    -- Core Complex Bus Switch ----------------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_core_busswitch_inst : entity neorv32.neorv32_busswitch
        port map(
            clk_i => clk_i,
            rstn_i => rstn_int,
            data_req_i => cpu_d_req,
            data_rsp_o => cpu_d_rsp,
            inst_req_i => cpu_i_req,
            inst_rsp_o => cpu_i_rsp,
            peri_req_o => core_req,
            peri_rsp_i => core_rsp
        );

    -- ****************************************************************************************************************************
    -- Bus System
    -- ****************************************************************************************************************************

    -- Bus Keeper (BUSKEEPER) -----------------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_bus_keeper_inst : entity neorv32.neorv32_bus_keeper
        port map(
            clk_i => clk_i,
            rstn_i => rstn_int,
            cpu_req_i => io_req,
            cpu_rsp_o => rsp_bus(DEV_BUSKEEPER),
            bus_req_i => soc_req,
            bus_rsp_i => soc_rsp,
            bus_err_o => bus_error,
            bus_tmo_i => ext_timeout,
            bus_ext_i => ext_access,
            bus_xip_i => xip_access
        );

    -- global bus response ---
    bus_response : process (rsp_bus)
        variable tmp_v : bus_rsp_t;
    begin
        tmp_v := rsp_terminate_c;
        for i in rsp_bus'range loop -- OR all response signals
            tmp_v.data := tmp_v.data or rsp_bus(i).data;
            tmp_v.ack := tmp_v.ack or rsp_bus(i).ack;
            tmp_v.err := tmp_v.err or rsp_bus(i).err;
        end loop;
        soc_rsp <= tmp_v;
    end process;

    -- central SoC bus --
    soc_req <= core_req;
    core_rsp.data <= soc_rsp.data;
    core_rsp.ack <= soc_rsp.ack;
    core_rsp.err <= bus_error; -- global bus error (buskeeper -> core)

    -- ****************************************************************************************************************************
    -- Memory System
    -- ****************************************************************************************************************************

    -- Processor-Internal Instruction Memory (IMEM) -------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_int_imem_inst : entity neorv32.neorv32_imem
        generic map(
            IMEM_BASE => imem_base_c,
            IMEM_SIZE => MEM_INT_IMEM_SIZE,
            IMEM_AS_IROM => not true
        )
        port map(
            clk_i => clk_i,
            bus_req_i => soc_req,
            bus_rsp_o => rsp_bus(DEV_IMEM)
        );

    -- Processor-Internal Data Memory (DMEM) --------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_int_dmem_inst : entity neorv32.neorv32_dmem
        generic map(
            DMEM_BASE => dmem_base_c,
            DMEM_SIZE => MEM_INT_DMEM_SIZE
        )
        port map(
            clk_i => clk_i,
            bus_req_i => soc_req,
            bus_rsp_o => rsp_bus(DEV_DMEM)
        );

    -- Processor-Internal Bootloader ROM (BOOTROM) --------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_boot_rom_inst : entity neorv32.neorv32_boot_rom
        generic map(
            BOOTROM_BASE => boot_rom_base_c
        )
        port map(
            clk_i => clk_i,
            bus_req_i => soc_req,
            bus_rsp_o => rsp_bus(DEV_BOOTROM)
        );

    neorv32_spi_inst: entity neorv32.neorv32_spi
        generic map (
            IO_SPI_FIFO => 1
        )
        port map (
            clk_i       => clk_i,
            rstn_i      => rstn_int,
            bus_req_i   => io_req,
            bus_rsp_o   => rsp_bus(DEV_SPI),
            clkgen_en_o => spi_cg_en,
            clkgen_i    => clk_gen,
            spi_clk_o   => xip_clk_o,
            spi_dat_o   => xip_dat_o,
            spi_dat_i   => xip_dat_i,
            spi_csn_o   => spi_csn,
            irq_o       => spi_irq
    );

    xip_csn_o <= spi_csn(0);

    -- ****************************************************************************************************************************
    -- IO/Peripheral Modules
    -- ****************************************************************************************************************************

    -- IO Gateway -----------------------------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    io_gateway : process (soc_req, io_acc)
    begin
        io_req <= soc_req;
        io_req.re <= io_acc and soc_req.re and (not soc_req.src); -- PMA: read access only from data interface
        io_req.we <= io_acc and soc_req.we and and_reduce_f(soc_req.ben); -- PMA: full-word write accesses only
    end process io_gateway;

    -- IO access? --
    io_acc <= '1' when (soc_req.addr(31) = '1') and (soc_req.addr(13) = '1') else
              '0';

    -- General Purpose Input/Output Port (GPIO) -----------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_gpio_inst : entity neorv32.neorv32_gpio
        generic map(
            GPIO_NUM => IO_GPIO_NUM
        )
        port map(
            -- host access --
            clk_i => clk_i,
            rstn_i => rstn_int,
            bus_req_i => io_req,
            bus_rsp_o => rsp_bus(DEV_GPIO),
            gpio_o => gpio_o,
            gpio_i => gpio_i
        );

    -- Watch Dog Timer (WDT) ------------------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    rstn_wdt <= '1';

    -- Primary Universal Asynchronous Receiver/Transmitter (UART0) ----------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_uart0_inst : entity neorv32.neorv32_uart
        generic map(
            UART_PRIMARY => true,
            UART_RX_FIFO => 1,
            UART_TX_FIFO => 1
        )
        port map(
            clk_i => clk_i,
            rstn_i => rstn_int,
            bus_req_i => io_req,
            bus_rsp_o => rsp_bus(DEV_UART0),
            clkgen_en_o => uart0_cg_en,
            clkgen_i => clk_gen,
            uart_txd_o => uart0_txd_o,
            uart_rxd_i => uart0_rxd_i,
            uart_rts_o => uart0_rts_o,
            uart_cts_i => uart0_cts_i,
            irq_rx_o => uart0_rx_irq,
            irq_tx_o => uart0_tx_irq
        );

    -- External Interrupt Controller (XIRQ) ---------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_xirq_inst : entity neorv32.neorv32_xirq
        generic map(
            XIRQ_NUM_CH => XIRQ_NUM_CH,
            XIRQ_TRIGGER_TYPE => XIRQ_TRIGGER_TYPE,
            XIRQ_TRIGGER_POLARITY => XIRQ_TRIGGER_POLARITY
        )
        port map(
            -- host access --
            clk_i => clk_i,
            rstn_i => rstn_int,
            bus_req_i => io_req,
            bus_rsp_o => rsp_bus(DEV_XIRQ),
            xirq_i => xirq_i,
            cpu_irq_o => xirq_irq
        );

    -- System Configuration Information Memory (SYSINFO) --------------------------------------
    -- -------------------------------------------------------------------------------------------
    neorv32_sysinfo_inst : entity neorv32.neorv32_sysinfo
        generic map(
            -- General --
            CLOCK_FREQUENCY => CLOCK_FREQUENCY,
            CUSTOM_ID => x"00000000",
            INT_BOOTLOADER_EN => true,
            -- Physical memory protection (PMP) --
            PMP_NUM_REGIONS => 0,
            -- internal Instruction memory --
            MEM_INT_IMEM_EN => true,
            MEM_INT_IMEM_SIZE => MEM_INT_IMEM_SIZE,
            -- Internal Data memory --
            MEM_INT_DMEM_EN => true,
            MEM_INT_DMEM_SIZE => MEM_INT_DMEM_SIZE,
            -- Instruction cache --
            ICACHE_EN => false,
            ICACHE_NUM_BLOCKS => 1,
            ICACHE_BLOCK_SIZE => 4,
            ICACHE_ASSOCIATIVITY => 1,
            -- Data cache --
            DCACHE_EN => false,
            DCACHE_NUM_BLOCKS => 1,
            DCACHE_BLOCK_SIZE => 4,
            -- External memory interface --
            MEM_EXT_EN => false,
            MEM_EXT_BIG_ENDIAN => false,
            -- On-Chip Debugger --
            ON_CHIP_DEBUGGER_EN => false,
            -- Processor peripherals --
            IO_GPIO_NUM => IO_GPIO_NUM,
            IO_MTIME_EN => false,
            IO_UART0_EN => true,
            IO_UART1_EN => false,
            IO_SPI_EN => false,
            IO_SDI_EN => false,
            IO_TWI_EN => false,
            IO_PWM_NUM_CH => 0,
            IO_WDT_EN => false,
            IO_TRNG_EN => false,
            IO_CFS_EN => false,
            IO_NEOLED_EN => false,
            IO_XIRQ_NUM_CH => XIRQ_NUM_CH,
            IO_GPTMR_EN => false,
            IO_XIP_EN => true,
            IO_ONEWIRE_EN => false,
            IO_DMA_EN => false
        )
        port map(
            clk_i => clk_i,
            bus_req_i => io_req,
            bus_rsp_o => rsp_bus(DEV_SYSINFO)
        );

end neorv32_top_rtl;