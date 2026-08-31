-- Every bag in one window: the inventory, the keyring, and the bank when you
-- are standing at one.
--
-- 3.3.5's interface has no such thing: it opens one ContainerFrame per bag and
-- has no sorting at all, both of which arrived years later. This client's own
-- bag window had a combined view and a Sort Bags button for a long time, but
-- that is drawn by the client - hand the bags to FrameXML and it goes away with
-- them. This is the same idea built out of the ordinary addon API, so it is
-- there whichever side is drawing.
--
-- Nothing here is special-cased. GetContainerNumSlots, GetContainerItemInfo,
-- PickupContainerItem, GetItemQualityColor and GameTooltip:SetBagItem are the
-- interface's own, and SortBags is this client's, which runs the sort the bag
-- window already used.

local COLS_MIN   = 6
local COLS_MAX   = 16
local SLOT       = 37      -- button size
local PAD        = 4       -- gap between slots
local EDGE       = 14      -- frame border inset
local TOP        = 62      -- room for the title, search box and buttons
local BOTTOM     = 34      -- room for the money line
local HEADER     = 16      -- room for a section's caption

-- The bags of each section, in the order they are drawn. The keyring and the
-- bank are only asked about when they have something to say - GetContainerNumSlots
-- answers zero for a bank bag while the bank is shut, so a section with no
-- slots simply does not appear.
local INVENTORY = {0, 1, 2, 3, 4}
local KEYRING   = {KEYRING_CONTAINER or -2}
local BANK      = {BANK_CONTAINER or -1, 5, 6, 7, 8, 9, 10, 11}

local buttons = {}         -- flat list, reused across redraws
local headers = {}         -- section captions, likewise
local search  = ""
local bankOpen = false

-- ── Saved settings ──────────────────────────────────────────────────────────
--
-- Position and column count only. Everything else is derived, and a setting
-- that can be worked out again is one more thing to migrate.

local function settings()
    if type(WoweeAllBagsSettings) ~= 'table' then WoweeAllBagsSettings = {} end
    if type(WoweeAllBagsSettings.columns) ~= 'number' then
        WoweeAllBagsSettings.columns = 10
    end
    return WoweeAllBagsSettings
end

local function columns()
    local n = settings().columns
    if n < COLS_MIN then return COLS_MIN end
    if n > COLS_MAX then return COLS_MAX end
    return n
end

-- ── The frame ───────────────────────────────────────────────────────────────

local f = CreateFrame("Frame", "WoweeAllBagsFrame", UIParent)
f:SetFrameStrata("HIGH")
f:SetToplevel(true)
f:SetMovable(true)
f:EnableMouse(true)
f:RegisterForDrag("LeftButton")
f:SetScript("OnDragStart", function(self) self:StartMoving() end)
f:SetScript("OnDragStop", function(self)
    self:StopMovingOrSizing()
    -- Where it was left, so it opens there next time.
    local s = settings()
    s.point, s.x, s.y = "CENTER", self:GetLeft(), self:GetBottom()
end)
f:SetBackdrop({
    bgFile   = "Interface\\DialogFrame\\UI-DialogBox-Background",
    edgeFile = "Interface\\DialogFrame\\UI-DialogBox-Border",
    tile = true, tileSize = 32, edgeSize = 32,
    insets = { left = 11, right = 12, top = 12, bottom = 11 },
})
f:Hide()

local title = f:CreateFontString(nil, "OVERLAY", "GameFontNormal")
title:SetPoint("TOP", f, "TOP", 0, -16)
title:SetText("All Bags")

local close = CreateFrame("Button", nil, f, "UIPanelCloseButton")
close:SetPoint("TOPRIGHT", f, "TOPRIGHT", -8, -8)

-- Sort. Disabled while the moves are still going out, because a sort is dozens
-- of swaps and the client sends them a tick at a time.
local sort = CreateFrame("Button", "WoweeAllBagsSort", f, "UIPanelButtonTemplate")
sort:SetWidth(64)
sort:SetHeight(21)
sort:SetPoint("TOPLEFT", f, "TOPLEFT", EDGE, -34)
sort:SetText("Sort")
sort:SetScript("OnClick", function()
    if SortBags then SortBags() end
end)

-- Search. Dims what does not match rather than hiding it, so the slots keep
-- their places and nothing jumps around under the cursor.
local box = CreateFrame("EditBox", "WoweeAllBagsSearch", f, "InputBoxTemplate")
box:SetWidth(150)
box:SetHeight(20)
box:SetPoint("LEFT", sort, "RIGHT", 12, 0)
box:SetAutoFocus(false)
box:SetScript("OnTextChanged", function(self)
    search = string.lower(self:GetText() or "")
    WoweeAllBags_Update()
end)
box:SetScript("OnEscapePressed", function(self) self:SetText("") self:ClearFocus() end)

-- Narrower and wider, which is the one bit of layout worth keeping between
-- sessions: how many columns suit a window depends on the screen it is on.
local wider = CreateFrame("Button", nil, f, "UIPanelButtonTemplate")
wider:SetWidth(22)
wider:SetHeight(21)
wider:SetPoint("TOPRIGHT", f, "TOPRIGHT", -34, -34)
wider:SetText(">")
wider:SetScript("OnClick", function()
    settings().columns = columns() + 1
    WoweeAllBags_Update()
end)

local narrower = CreateFrame("Button", nil, f, "UIPanelButtonTemplate")
narrower:SetWidth(22)
narrower:SetHeight(21)
narrower:SetPoint("RIGHT", wider, "LEFT", -2, 0)
narrower:SetText("<")
narrower:SetScript("OnClick", function()
    settings().columns = columns() - 1
    WoweeAllBags_Update()
end)

local counts = f:CreateFontString(nil, "OVERLAY", "GameFontHighlightSmall")
counts:SetPoint("BOTTOMLEFT", f, "BOTTOMLEFT", EDGE + 2, 14)

local money = CreateFrame("Frame", "WoweeAllBagsMoney", f, "SmallMoneyFrameTemplate")
money:SetPoint("BOTTOMRIGHT", f, "BOTTOMRIGHT", -EDGE, 10)

-- ── Slots ───────────────────────────────────────────────────────────────────

local function slotButton(index)
    local b = buttons[index]
    if b then return b end
    b = CreateFrame("Button", "WoweeAllBagsItem" .. index, f)
    b:SetWidth(SLOT)
    b:SetHeight(SLOT)
    b:RegisterForClicks("LeftButtonUp", "RightButtonUp")

    b.icon = b:CreateTexture(nil, "BACKGROUND")
    b.icon:SetAllPoints(b)

    -- The rarity ring. Drawn over the icon and under the count, coloured from
    -- GetItemQualityColor so it says the same thing the item's name does in a
    -- tooltip. Hidden for common and poor, which is what the real bag does:
    -- a border on everything is a border that says nothing.
    b.border = b:CreateTexture(nil, "ARTWORK")
    b.border:SetPoint("TOPLEFT", b, "TOPLEFT", -2, 2)
    b.border:SetPoint("BOTTOMRIGHT", b, "BOTTOMRIGHT", 2, -2)
    b.border:SetTexture("Interface\\Buttons\\UI-ActionButton-Border")
    b.border:SetBlendMode("ADD")
    b.border:Hide()

    b.count = b:CreateFontString(nil, "OVERLAY", "NumberFontNormal")
    b.count:SetPoint("BOTTOMRIGHT", b, "BOTTOMRIGHT", -2, 2)

    b:SetScript("OnEnter", function(self)
        if not self.bag then return end
        GameTooltip:SetOwner(self, "ANCHOR_RIGHT")
        GameTooltip:SetBagItem(self.bag, self.slot)
        GameTooltip:Show()
    end)
    b:SetScript("OnLeave", function() GameTooltip:Hide() end)
    b:SetScript("OnClick", function(self, button)
        if not self.bag then return end
        if button == "RightButton" then UseContainerItem(self.bag, self.slot)
        else                            PickupContainerItem(self.bag, self.slot) end
        WoweeAllBags_Update()
    end)

    buttons[index] = b
    return b
end

local function sectionHeader(index)
    local h = headers[index]
    if h then return h end
    h = f:CreateFontString(nil, "OVERLAY", "GameFontNormalSmall")
    headers[index] = h
    return h
end

-- ── Redraw ──────────────────────────────────────────────────────────────────

--- The sections with anything in them, in drawing order.
---
--- Asked every redraw rather than remembered: a bank bag bought while the
--- window is open has slots from that moment, and the keyring answers zero
--- until the player owns one.
local function visibleSections()
    local out = {}
    local function add(name, bags)
        local total = 0
        for _, bag in ipairs(bags) do
            total = total + (GetContainerNumSlots(bag) or 0)
        end
        if total > 0 then out[#out + 1] = {name = name, bags = bags} end
    end
    add("Inventory", INVENTORY)
    add("Keyring", KEYRING)
    if bankOpen then add("Bank", BANK) end
    return out
end

function WoweeAllBags_Update()
    if not f:IsShown() then return end

    local cols = columns()
    local shown, used, free = 0, 0, 0
    local row, usedHeaders = 0, 0
    local sections = visibleSections()

    for si, section in ipairs(sections) do
        -- A caption for each, but only once there is more than one to tell
        -- apart: a window showing nothing but the inventory does not need to
        -- say so.
        if #sections > 1 then
            usedHeaders = usedHeaders + 1
            local h = sectionHeader(usedHeaders)
            h:ClearAllPoints()
            h:SetPoint("TOPLEFT", f, "TOPLEFT",
                       EDGE, -(TOP + row * (SLOT + PAD) + (usedHeaders - 1) * HEADER))
            h:SetText(section.name)
            h:Show()
            row = row + HEADER / (SLOT + PAD)
        end

        local col = 0
        for _, bag in ipairs(section.bags) do
            local size = GetContainerNumSlots(bag) or 0
            for slot = 1, size do
                shown = shown + 1
                local b = slotButton(shown)
                b.bag, b.slot = bag, slot

                b:ClearAllPoints()
                b:SetPoint("TOPLEFT", f, "TOPLEFT",
                           EDGE + col * (SLOT + PAD),
                           -(TOP + row * (SLOT + PAD) + usedHeaders * HEADER))

                local texture, count, _, quality = GetContainerItemInfo(bag, slot)
                if texture then
                    used = used + 1
                    b.icon:SetTexture(texture)
                    b.count:SetText((count and count > 1) and count or "")

                    -- Dimmed rather than hidden while a search is running, so a
                    -- slot never moves out from under the cursor mid-look.
                    local match = true
                    if search ~= "" then
                        local link = GetContainerItemLink(bag, slot)
                        local name = link and string.match(link, "%[(.-)%]") or ""
                        match = string.find(string.lower(name), search, 1, true) ~= nil
                    end
                    b.icon:SetVertexColor(1, 1, 1, match and 1 or 0.25)

                    -- Uncommon and up get a ring; white and grey do not.
                    if quality and quality > 1 and GetItemQualityColor then
                        local r, g, bl = GetItemQualityColor(quality)
                        b.border:SetVertexColor(r, g, bl, match and 1 or 0.25)
                        b.border:Show()
                    else
                        b.border:Hide()
                    end
                else
                    free = free + 1
                    b.icon:SetTexture("Interface\\Buttons\\UI-EmptySlot-White")
                    b.icon:SetVertexColor(1, 1, 1, 0.35)
                    b.count:SetText("")
                    b.border:Hide()
                end
                b:Show()

                col = col + 1
                if col >= cols then col = 0; row = row + 1 end
            end
        end
        -- The next section starts on its own row.
        if col > 0 then row = row + 1 end
    end

    -- Anything left from a larger set of bags stays built but out of the way.
    for i = shown + 1, #buttons do buttons[i]:Hide() end
    for i = usedHeaders + 1, #headers do headers[i]:Hide() end

    local rows = math.max(1, math.ceil(row))
    f:SetWidth(EDGE * 2 + cols * (SLOT + PAD) - PAD)
    f:SetHeight(TOP + rows * (SLOT + PAD) - PAD + BOTTOM + usedHeaders * HEADER)

    counts:SetText(used .. " used, " .. free .. " free")
    if MoneyFrame_Update then MoneyFrame_Update("WoweeAllBagsMoney", GetMoney()) end
    if sort.SetEnabled then
        sort:SetEnabled(not (IsSortingBags and IsSortingBags()))
    end
end

-- ── Events and the way in ───────────────────────────────────────────────────

f:SetScript("OnEvent", function(self, event)
    if event == "BANKFRAME_OPENED" then
        bankOpen = true
    elseif event == "BANKFRAME_CLOSED" then
        bankOpen = false
    end
    WoweeAllBags_Update()
end)
f:RegisterEvent("BAG_UPDATE")
f:RegisterEvent("ITEM_LOCK_CHANGED")
f:RegisterEvent("PLAYER_MONEY")
f:RegisterEvent("BANKFRAME_OPENED")
f:RegisterEvent("BANKFRAME_CLOSED")
f:RegisterEvent("PLAYERBANKSLOTS_CHANGED")

function WoweeAllBags_Toggle()
    if f:IsShown() then
        f:Hide()
    else
        local s = settings()
        f:ClearAllPoints()
        if s.x and s.y then
            f:SetPoint("BOTTOMLEFT", UIParent, "BOTTOMLEFT", s.x, s.y)
        else
            f:SetPoint("CENTER", UIParent, "CENTER", 0, 0)
        end
        f:Show()
        WoweeAllBags_Update()
    end
end

SLASH_WOWEEALLBAGS1 = "/allbags"
SLASH_WOWEEALLBAGS2 = "/bags"
SlashCmdList["WOWEEALLBAGS"] = WoweeAllBags_Toggle
