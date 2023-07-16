-- #################################################################################################
-- # << NEORV32 - Bus Switch >>                                                                    #
-- # ********************************************************************************************* #
-- # Allows to access a single peripheral bus ("p_bus") by two controller ports. Controller port A #
-- # ("ca_bus") has priority over controller port B ("cb_bus").                                    #
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

entity neorv32_busswitch is
    port (
        clk_i : in std_ulogic; -- global clock, rising edge
        rstn_i : in std_ulogic; -- global reset, low-active, async
        data_req_i : in bus_req_t; -- host data port: request bus
        data_rsp_o : out bus_rsp_t; -- host data port: response bus
        inst_req_i : in inst_req_t; -- host inst port: request bus
        inst_rsp_o : out bus_rsp_t; -- host inst port: response bus
        peri_req_o : out bus_req_t; -- device port request bus
        peri_rsp_i : in bus_rsp_t -- device port response bus
    );
end neorv32_busswitch;

architecture neorv32_busswitch_rtl of neorv32_busswitch is

    constant PORT_DATA_READ_ONLY : boolean := false;
    constant PORT_INST_READ_ONLY : boolean := true;

    -- access requests --
    signal data_rd_req_buf, data_wr_req_buf : std_ulogic;
    signal inst_rd_req_buf, inst_wr_req_buf : std_ulogic;
    signal data_req_current, data_req_pending : std_ulogic;
    signal inst_req_current, inst_req_pending : std_ulogic;

    -- internal bus lines --
    signal data_bus_ack, inst_bus_ack : std_ulogic;
    signal data_bus_err, inst_bus_err : std_ulogic;
    signal peri_bus_we, peri_bus_re : std_ulogic;

    -- access arbiter --
    type arbiter_state_t is (IDLE, DATA_BUSY, DATA_RETIRE, INST_BUSY, INST_RETIRE);
    type arbiter_t is record
        state : arbiter_state_t;
        state_nxt : arbiter_state_t;
        bus_sel : std_ulogic;
        re_trig : std_ulogic;
        we_trig : std_ulogic;
    end record;
    signal arbiter : arbiter_t;

    signal cur_addr : caddr_t;
    signal is_data : std_ulogic;
    signal is_boot : std_ulogic;
    signal is_peri : std_ulogic;

    signal space : space_t;

begin

    -- Access Arbiter -------------------------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    arbiter_sync : process (rstn_i, clk_i)
    begin
        if (rstn_i = '0') then
            arbiter.state <= IDLE;
            data_rd_req_buf <= '0';
            data_wr_req_buf <= '0';
            inst_rd_req_buf <= '0';
            inst_wr_req_buf <= '0';
        elsif rising_edge(clk_i) then
            arbiter.state <= arbiter.state_nxt;
            -- port A requests --
            data_rd_req_buf <= (data_rd_req_buf or data_req_i.re) and (not (data_bus_err or data_bus_ack));
            data_wr_req_buf <= (data_wr_req_buf or data_req_i.we) and (not (data_bus_err or data_bus_ack)) and bool_to_ulogic_f(PORT_DATA_READ_ONLY = false);
            -- port B requests --
            inst_rd_req_buf <= (inst_rd_req_buf or inst_req_i.re) and (not (inst_bus_err or inst_bus_ack));
            inst_wr_req_buf <= (inst_wr_req_buf or inst_req_i.we) and (not (inst_bus_err or inst_bus_ack)) and bool_to_ulogic_f(PORT_INST_READ_ONLY = false);
        end if;
    end process arbiter_sync;

    -- any current requests? --
    data_req_current <= (data_req_i.re or data_req_i.we) when (PORT_DATA_READ_ONLY = false) else
        data_req_i.re;
    inst_req_current <= (inst_req_i.re or inst_req_i.we) when (PORT_INST_READ_ONLY = false) else
        inst_req_i.re;

    -- any pending requests? --
    data_req_pending <= (data_rd_req_buf or data_wr_req_buf) when (PORT_DATA_READ_ONLY = false) else
        data_rd_req_buf;
    inst_req_pending <= (inst_rd_req_buf or inst_wr_req_buf) when (PORT_INST_READ_ONLY = false) else
        inst_rd_req_buf;

    -- FSM --
    arbiter_comb : process (arbiter, data_req_current, inst_req_current, data_req_pending, inst_req_pending,
        data_rd_req_buf, data_wr_req_buf, inst_rd_req_buf, inst_wr_req_buf, peri_rsp_i)
    begin
        -- arbiter defaults --
        arbiter.state_nxt <= arbiter.state;
        arbiter.bus_sel <= '0';
        arbiter.we_trig <= '0';
        arbiter.re_trig <= '0';

        -- state machine --
        case arbiter.state is

            when IDLE => -- wait for requests
                -- ------------------------------------------------------------
                if (data_req_current = '1') then -- current request from port A?
                    arbiter.bus_sel <= '0';
                    arbiter.state_nxt <= DATA_BUSY;
                elsif (data_req_pending = '1') then -- pending request from port A?
                    arbiter.bus_sel <= '0';
                    arbiter.state_nxt <= DATA_RETIRE;
                elsif (inst_req_current = '1') then -- pending request from port B?
                    arbiter.bus_sel <= '1';
                    arbiter.state_nxt <= INST_BUSY;
                elsif (inst_req_pending = '1') then -- current request from port B?
                    arbiter.bus_sel <= '1';
                    arbiter.state_nxt <= INST_RETIRE;
                end if;

            when DATA_BUSY => -- port A pending access
                -- ------------------------------------------------------------
                arbiter.bus_sel <= '0'; -- access from port A
                if (peri_rsp_i.err = '1') or (peri_rsp_i.ack = '1') then
                    -- [COMMENT NOTE] Direct return to IDLE to further promote port A access requests.
                    --        if (inst_req_pending = '1') or (inst_req_current = '1') then -- any request from B?
                    --          arbiter.state_nxt <= B_RETIRE;
                    --        else
                    arbiter.state_nxt <= IDLE;
                    --        end if;
                end if;

            when DATA_RETIRE => -- retire port A pending access
                -- ------------------------------------------------------------
                arbiter.bus_sel <= '0'; -- access from port A
                arbiter.we_trig <= data_wr_req_buf;
                arbiter.re_trig <= data_rd_req_buf;
                arbiter.state_nxt <= DATA_BUSY;

            when INST_BUSY => -- port B pending access
                -- ------------------------------------------------------------
                arbiter.bus_sel <= '1'; -- access from port B
                if (peri_rsp_i.err = '1') or (peri_rsp_i.ack = '1') then
                    if (data_req_pending = '1') or (data_req_current = '1') then -- any request from A?
                        arbiter.state_nxt <= DATA_RETIRE;
                    else
                        arbiter.state_nxt <= IDLE;
                    end if;
                end if;

            when INST_RETIRE => -- retire port B pending access
                -- ------------------------------------------------------------
                arbiter.bus_sel <= '1'; -- access from port B
                arbiter.we_trig <= inst_wr_req_buf;
                arbiter.re_trig <= inst_rd_req_buf;
                arbiter.state_nxt <= INST_BUSY;

            when others => -- undefined
                -- ------------------------------------------------------------
                arbiter.state_nxt <= IDLE;

        end case;
    end process arbiter_comb;

    -- Peripheral Bus Switch ------------------------------------------------------------------
    -- -------------------------------------------------------------------------------------------
    cur_addr <= data_req_i.addr when (arbiter.bus_sel = '0') else
                  "0" & inst_req_i.addr;

    space <= space_of(cur_addr);

    peri_req_o.addr <= data_req_i.addr when (arbiter.bus_sel = '0') else
                       "0" & inst_req_i.addr;

    peri_req_o.data <= inst_req_i.data when (PORT_DATA_READ_ONLY = true) else
                       data_req_i.data when (PORT_INST_READ_ONLY = true) else
                       data_req_i.data when (arbiter.bus_sel = '0') else
                       inst_req_i.data;

    peri_req_o.ben <= inst_req_i.ben when (PORT_DATA_READ_ONLY = true) else
                      data_req_i.ben when (PORT_INST_READ_ONLY = true) else
                      data_req_i.ben when (arbiter.bus_sel = '0') else
                      inst_req_i.ben;

    peri_req_o.src <= data_req_i.src when (arbiter.bus_sel = '0') else
                      inst_req_i.src;

    peri_bus_we <= data_req_i.we when (arbiter.bus_sel = '0') else
                   inst_req_i.we;
    peri_bus_re <= data_req_i.re when (arbiter.bus_sel = '0') else
                   inst_req_i.re;
    peri_req_o.we <= peri_bus_we or arbiter.we_trig;
    peri_req_o.re <= peri_bus_re or arbiter.re_trig;

    data_rsp_o.data <= peri_rsp_i.data;
    inst_rsp_o.data <= peri_rsp_i.data;

    data_bus_ack <= peri_rsp_i.ack when (arbiter.bus_sel = '0') else
                    '0';
    inst_bus_ack <= peri_rsp_i.ack when (arbiter.bus_sel = '1') else
                    '0';
    data_rsp_o.ack <= data_bus_ack;
    inst_rsp_o.ack <= inst_bus_ack;

    data_bus_err <= peri_rsp_i.err when (arbiter.bus_sel = '0') else
                    '0';
    inst_bus_err <= peri_rsp_i.err when (arbiter.bus_sel = '1') else
                    '0';
    data_rsp_o.err <= data_bus_err;
    inst_rsp_o.err <= inst_bus_err;

end neorv32_busswitch_rtl;