library ieee;
use ieee.std_logic_1164.all;

package defBusAddressMap is

  constant kCurrentVersion      : std_logic_vector(31 downto 0):= x"70410201";
  constant kNumModules          : natural:= 8;

  constant kWidthModuleID       : positive:=4;

  -- Local Bus index ------------------------------------------------------
  subtype ModuleID is integer range -1 to kNumModules-1;
  type Leaf is record
    ID : ModuleID;
  end record;

  type Binder is array (integer range <>) of Leaf;
  constant kASIC  : Leaf := (ID => 0);
  constant kIOM   : Leaf := (ID => 1);

  constant kMUTIL : Leaf := (ID => 2);
  constant kTDC   : Leaf := (ID => 3);

  constant kSCR   : Leaf := (ID => 4);
  constant kBIAS  : Leaf := (ID => 5);
  --constant kMIG   : Leaf := (ID => 6);
  constant kSDS	  : Leaf := (ID => 6);
  constant kFMP   : Leaf := (ID => 7);
  constant kDummy : Leaf := (ID => -1);

  function GetID(mid_ext_bus: std_logic_vector(kWidthModuleID-1 downto 0))  return ModuleID;
  constant kMidBCT          : std_logic_vector(kWidthModuleID-1 downto 0) := "1110";

end package defBusAddressMap;

package body defBusAddressMap is

  -- Local Module ID ------------------------------------------------------
  -- Module ID Map
  -- <Module ID : 31-28> + <Local Address 27 - 16>
  -- SiTCP:		  1111 (Reserved)
  -- MidBCT:		1110
  function GetID(mid_ext_bus: std_logic_vector(kWidthModuleID-1 downto 0)) return ModuleID is
  begin
    case mid_ext_bus is
      when "0000"   => return kMUTIL.ID;
      when "0011"   => return kASIC.ID;
      when "0100"   => return kIOM.ID;
      when "0101"   => return kTDC.ID;

      when "1000"   => return kSCR.ID;
      when "1001"   => return kBIAS.ID;
      --when "1010"   => return kMIG.ID;
      when "1100"   => return kSDS.ID;
      when "1101"   => return kFMP.ID;
      when others   => return kDummy.ID;
    end case;
  end function GetID;

end package body defBusAddressMap;
