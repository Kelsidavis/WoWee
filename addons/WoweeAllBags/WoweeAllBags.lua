-- Every carried bag in one window, and the bank in a window of its own.
--
-- The bank used to be a third section of the bag window, reached by a Bank
-- button where the caption would be, so opening the bank replaced the view of
-- what you were carrying with what you had stored - which is the one moment
-- both are wanted at once. They are two windows now, each with its own
-- position and its own width, and the bank's opens where the bank does.
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
local TOP        = 40      -- room for the one row of controls
local BOTTOM     = 34      -- room for the money line
local HEADER     = 16      -- room for a section's caption

-- The bags of each section, in the order they are drawn. A section is only
-- drawn when it has something to say: the keyring answers zero slots until the
-- player owns one, and a bank bag slot that has never been bought answers zero
-- too, so a section with no slots simply does not appear.
local INVENTORY = {0, 1, 2, 3, 4}
local KEYRING   = {KEYRING_CONTAINER or -2}
local BANK      = {BANK_CONTAINER or -1, 5, 6, 7, 8, 9, 10, 11}

-- Declared here and defined beside the overrides they belong to, which is
-- where the bag window is opened and closed on the interface's behalf.
local openOurs, closeOurs

-- ── Saved settings ──────────────────────────────────────────────────────────
--
-- Position and column count, per window. Everything else is derived, and a
-- setting that can be worked out again is one more thing to migrate.
--
-- The bag window keeps its at the root, where it has always written them, so a
-- player who has dragged it somewhere keeps that place. The bank's live under
-- a key of their own: two windows that opened at one remembered position would
-- open on top of each other.

local function settings()
    if type(WoweeAllBagsSettings) ~= 'table' then WoweeAllBagsSettings = {} end
    if type(WoweeAllBagsSettings.columns) ~= 'number' then
        WoweeAllBagsSettings.columns = 10
    end
    if type(WoweeAllBagsSettings.bank) ~= 'table' then
        WoweeAllBagsSettings.bank = {}
    end
    if type(WoweeAllBagsSettings.bank.columns) ~= 'number' then
        WoweeAllBagsSettings.bank.columns = 10
    end
    -- A corner this window was never anchored by, saved by an earlier version
    -- and read by nothing. Dropped here so it stops being written back.
    WoweeAllBagsSettings.point = nil
    return WoweeAllBagsSettings
end

--- Put them on disk now, rather than trusting the exit to do it.
---
--- SavedVariables are written when the interface unloads, which is a logout or
--- a tidy quit. A client killed, rebuilt under itself or crashed reaches none
--- of those, so a window dragged into place was back where it started on the
--- next start - the setting was never kept, only remembered. Called where
--- something worth keeping has just changed; the client writes a few hundred
--- bytes and nothing here calls it per frame.
local function remember()
    if WoweeSaveVariables then WoweeSaveVariables() end
end

-- ── The client's own controls ───────────────────────────────────────────────
--
-- Interface > Bags in the WoWee settings has had three of these since before
-- the bags were handed to FrameXML: "Separate bag windows", "Show keyring" and
-- "Bag scale". They drove this client's own bag window, and that window went
-- when FrameXML took the bags - so all three have been controls wired to
-- nothing. They mean exactly what these windows do, so this reads them rather
-- than growing a second set of preferences beside them.
--
-- Strings both ways, as a CVar is. A client too old to answer leaves the
-- fallback, which is what the schema says the default is.

local function clientSetting(key, fallback)
    if not WoweeGetSetting then return fallback end
    local v = WoweeGetSetting(key)
    if v == nil or v == '' then return fallback end
    return v
end

local function clientFlag(key, fallback)
    return clientSetting(key, fallback and '1' or '0') ~= '0'
end

--- One window, or FrameXML's one per bag. The player's answer to that decides
--- whether this replaces the bags at all - see the overrides at the end.
local function separateBags()
    return clientFlag('separatebags', true)
end

local function showKeyring()
    return clientFlag('showkeyring', true)
end

-- ── A window ────────────────────────────────────────────────────────────────
--
-- Both windows are the same thing over a different set of bags: a grid of
-- slots, a search box, the two buttons that make it narrower or wider, and a
-- line saying how full it is. What differs is which bags they draw, where they
-- remember being, and whether they carry the money line and the Sort button -
-- SortBags sorts what the player is carrying, so on the bank it would have
-- been a button that quietly sorted a different window's contents.
--
-- Written as one builder rather than two files because every fix to a slot -
-- the rarity ring, the drag, HandleModifiedItemClick - has to reach both, and
-- a second copy is where that stops happening.

local windows = {}

local function newWindow(spec)
    local w = {
        buttons = {},
        headers = {},
        search  = "",
        sections = spec.sections,
        store = spec.store,
    }

    local function columns()
        local n = w.store().columns or 10
        if n < COLS_MIN then return COLS_MIN end
        if n > COLS_MAX then return COLS_MAX end
        return n
    end

    local f = CreateFrame("Frame", spec.frameName, UIParent)
    w.frame = f
    f:SetFrameStrata("HIGH")
    f:SetToplevel(true)
    f:SetMovable(true)
    f:EnableMouse(true)
    f:RegisterForDrag("LeftButton")
    f:SetScript("OnDragStart", function(self) self:StartMoving() end)
    f:SetScript("OnDragStop", function(self)
        self:StopMovingOrSizing()
        -- Where it was left, so it opens there next time. The corner it is
        -- stored by is the corner it is anchored to when it opens - see the
        -- SetPoint in Toggle below.
        local st = w.store()
        st.x, st.y = self:GetLeft(), self:GetBottom()
        remember()
    end)
    f:SetBackdrop({
        bgFile   = "Interface\\DialogFrame\\UI-DialogBox-Background",
        edgeFile = "Interface\\DialogFrame\\UI-DialogBox-Border",
        tile = true, tileSize = 32, edgeSize = 32,
        insets = { left = 11, right = 12, top = 12, bottom = 11 },
    })
    f:Hide()

    local close = CreateFrame("Button", nil, f, "UIPanelCloseButton")
    close:SetPoint("TOPRIGHT", f, "TOPRIGHT", -8, -8)

    -- The caption. The bag window has none - "All Bags" over a window full of
    -- bags said nothing the window did not - but two windows on screen at once
    -- do need telling apart, and the bank's is the one that appears without
    -- being asked for.
    local leftAnchor = f
    local leftX = EDGE
    if spec.title then
        local caption = f:CreateFontString(nil, "OVERLAY", "GameFontNormal")
        caption:SetPoint("TOPLEFT", f, "TOPLEFT", EDGE, -14)
        caption:SetText(spec.title)
        leftAnchor = caption
        leftX = 8
    end

    local sort
    if spec.sort then
        -- Disabled while the moves are still going out, because a sort is
        -- dozens of swaps and the client sends them a tick at a time.
        sort = CreateFrame("Button", spec.sortName, f, "UIPanelButtonTemplate")
        sort:SetWidth(64)
        sort:SetHeight(21)
        sort:SetPoint("TOPLEFT", f, "TOPLEFT", EDGE, -12)
        sort:SetText("Sort")
        sort:SetScript("OnClick", function()
            if SortBags then SortBags() end
        end)
        leftAnchor = sort
        leftX = 12
    end

    -- Search. Dims what does not match rather than hiding it, so the slots keep
    -- their places and nothing jumps around under the cursor.
    local box = CreateFrame("EditBox", spec.searchName, f, "InputBoxTemplate")
    box:SetWidth(150)
    box:SetHeight(20)
    if leftAnchor == f then
        box:SetPoint("TOPLEFT", f, "TOPLEFT", leftX + 8, -12)
    else
        box:SetPoint("LEFT", leftAnchor, "RIGHT", leftX, 0)
    end
    box:SetAutoFocus(false)
    box:SetScript("OnTextChanged", function(self)
        w.search = string.lower(self:GetText() or "")
        w.Update()
    end)
    box:SetScript("OnEscapePressed", function(self) self:SetText("") self:ClearFocus() end)

    -- Narrower and wider, which is the one bit of layout worth keeping between
    -- sessions: how many columns suit a window depends on the screen it is on.
    local wider = CreateFrame("Button", nil, f, "UIPanelButtonTemplate")
    wider:SetWidth(22)
    wider:SetHeight(21)
    wider:SetPoint("TOPRIGHT", f, "TOPRIGHT", -34, -12)
    wider:SetText(">")
    wider:SetScript("OnClick", function()
        w.store().columns = columns() + 1
        remember()
        w.Update()
    end)

    local narrower = CreateFrame("Button", nil, f, "UIPanelButtonTemplate")
    narrower:SetWidth(22)
    narrower:SetHeight(21)
    narrower:SetPoint("RIGHT", wider, "LEFT", -2, 0)
    narrower:SetText("<")
    narrower:SetScript("OnClick", function()
        w.store().columns = columns() - 1
        remember()
        w.Update()
    end)

    local counts = f:CreateFontString(nil, "OVERLAY", "GameFontHighlightSmall")
    counts:SetPoint("BOTTOMLEFT", f, "BOTTOMLEFT", EDGE + 2, 14)

    local money
    if spec.money then
        money = CreateFrame("Frame", spec.moneyName, f, "SmallMoneyFrameTemplate")
        money:SetPoint("BOTTOMRIGHT", f, "BOTTOMRIGHT", -EDGE, 10)
    end

    -- ── Slots ───────────────────────────────────────────────────────────────

    local function slotButton(index)
        local b = w.buttons[index]
        if b then return b end
        b = CreateFrame("Button", spec.itemPrefix .. index, f)
        b:SetWidth(SLOT)
        b:SetHeight(SLOT)
        b:RegisterForClicks("LeftButtonUp", "RightButtonUp")
        -- Dragged as well as clicked. Clicking to pick up and clicking again to
        -- drop is how the real bags work and it was all this had; dragging one
        -- slot onto another is the other half, and without it a press that
        -- moved at all did nothing. A drag belongs to the frame the press
        -- landed on, so this takes it from the window's own drag rather than
        -- fighting it.
        b:RegisterForDrag("LeftButton")

        b.icon = b:CreateTexture(nil, "BACKGROUND")
        b.icon:SetAllPoints(b)

        -- The rarity ring. Drawn over the icon and under the count, coloured
        -- from GetItemQualityColor so it says the same thing the item's name
        -- does in a tooltip. Hidden for common and poor, which is what the real
        -- bag does: a border on everything is a border that says nothing.
        -- Much larger than the slot and centred on it, because this texture is
        -- a soft glow with most of its size given over to the falloff. Held to
        -- the slot's own bounds the faded part is cropped away and what is left
        -- is a flat coloured square sitting behind the icon.
        b.border = b:CreateTexture(nil, "ARTWORK")
        b.border:SetWidth(SLOT + 30)
        b.border:SetHeight(SLOT + 30)
        b.border:SetPoint("CENTER", b, "CENTER", 0, 0)
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
        b:SetScript("OnDragStart", function(self)
            if not self.bag then return end
            PickupContainerItem(self.bag, self.slot)
            WoweeAllBags_Update()
        end)
        -- Dropping onto a slot is the same call: it puts down what the cursor
        -- is holding, or swaps it with what is already there. Both windows
        -- update, because a bank deposit changes what is carried too.
        b:SetScript("OnReceiveDrag", function(self)
            if not self.bag then return end
            PickupContainerItem(self.bag, self.slot)
            WoweeAllBags_Update()
        end)
        b:SetScript("OnClick", function(self, button)
            if not self.bag then return end
            -- Shift to put the item in chat, ctrl to try it on. Both are
            -- HandleModifiedItemClick's to answer - it is what every item
            -- button in the interface asks first, and it says whether it took
            -- the click. Without it a shift-click picked the item up instead
            -- of linking it.
            if HandleModifiedItemClick and
               HandleModifiedItemClick(GetContainerItemLink(self.bag, self.slot)) then
                return
            end
            if button == "RightButton" then UseContainerItem(self.bag, self.slot)
            else                            PickupContainerItem(self.bag, self.slot) end
            WoweeAllBags_Update()
        end)

        w.buttons[index] = b
        return b
    end

    local function sectionHeader(index)
        local h = w.headers[index]
        if h then return h end
        h = f:CreateFontString(nil, "OVERLAY", "GameFontNormalSmall")
        w.headers[index] = h
        return h
    end

    -- ── Redraw ──────────────────────────────────────────────────────────────

    function w.Update()
        if not f:IsShown() then return end

        local cols = columns()
        local shown, used, free = 0, 0, 0
        local row, usedHeaders = 0, 0
        local sections = w.sections()

        for si, section in ipairs(sections) do
            -- A caption for each, but only once there is more than one to tell
            -- apart: a window showing nothing but the inventory does not need
            -- to say so.
            if #sections > 1 then
                usedHeaders = usedHeaders + 1
                local h = sectionHeader(usedHeaders)
                h:ClearAllPoints()
                h:SetPoint("TOPLEFT", f, "TOPLEFT", EDGE,
                           -(TOP + row * (SLOT + PAD) + (usedHeaders - 1) * HEADER))
                h:SetText(section.name)
                h:Show()
                row = row + HEADER / (SLOT + PAD)
            end

            local col = 0
            for _, bag in ipairs(section.bags) do
                local slots = GetContainerNumSlots(bag) or 0
                for slot = 1, slots do
                    shown = shown + 1
                    local b = slotButton(shown)
                    b.bag, b.slot = bag, slot
                    b:ClearAllPoints()
                    b:SetPoint("TOPLEFT", f, "TOPLEFT",
                               EDGE + col * (SLOT + PAD),
                               -(TOP + row * (SLOT + PAD) + usedHeaders * HEADER))
                    b:Show()

                    local texture, count, _, _, _, _, link = GetContainerItemInfo(bag, slot)
                    if texture then
                        used = used + 1
                        b.icon:SetTexture(texture)
                        b.icon:Show()
                        b.count:SetText((count and count > 1) and count or "")

                        -- Rarity, from the item's link. A quality the client
                        -- has not answered for yet leaves the ring hidden
                        -- rather than drawing a grey one.
                        local quality
                        if link and GetItemInfo then
                            local _, _, q = GetItemInfo(link)
                            quality = q
                        end
                        if quality and quality > 1 and GetItemQualityColor then
                            local r, g, bl = GetItemQualityColor(quality)
                            b.border:SetVertexColor(r, g, bl)
                            b.border:Show()
                        else
                            b.border:Hide()
                        end

                        -- The search dims what does not match. Compared
                        -- against the item's name out of its link, which is
                        -- the only name a slot has here.
                        local dim = false
                        if w.search ~= "" then
                            local name = link and string.match(link, "%[(.-)%]")
                            dim = not (name and string.find(string.lower(name),
                                                            w.search, 1, true))
                        end
                        b.icon:SetAlpha(dim and 0.25 or 1.0)
                        b.border:SetAlpha(dim and 0.25 or 1.0)
                    else
                        free = free + 1
                        b.icon:SetTexture(nil)
                        b.icon:Hide()
                        b.border:Hide()
                        b.count:SetText("")
                    end

                    col = col + 1
                    if col >= cols then
                        col = 0
                        row = row + 1
                    end
                end
            end
            if col > 0 then row = row + 1 end
        end

        -- Anything left from a larger set of bags stays built but out of the way.
        for i = shown + 1, #w.buttons do w.buttons[i]:Hide() end
        for i = usedHeaders + 1, #w.headers do w.headers[i]:Hide() end

        local rows = math.max(1, math.ceil(row))
        f:SetWidth(EDGE * 2 + cols * (SLOT + PAD) - PAD)
        f:SetHeight(TOP + rows * (SLOT + PAD) - PAD + BOTTOM + usedHeaders * HEADER)

        -- The size the player asked for, from the same control that used to
        -- size this client's own bag window.
        local scale = tonumber(clientSetting('bagscale', '1')) or 1
        if scale < 0.5 then scale = 0.5 elseif scale > 2 then scale = 2 end
        if f:GetScale() ~= scale then f:SetScale(scale) end

        counts:SetText(used .. " used, " .. free .. " free")
        if money and MoneyFrame_Update then
            MoneyFrame_Update(spec.moneyName, GetMoney())
        end
        if sort and sort.SetEnabled then
            sort:SetEnabled(not (IsSortingBags and IsSortingBags()))
        end
    end

    function w.Toggle()
        if f:IsShown() then
            f:Hide()
        else
            local st = w.store()
            f:ClearAllPoints()
            if st.x and st.y then
                f:SetPoint("BOTTOMLEFT", UIParent, "BOTTOMLEFT", st.x, st.y)
            else
                f:SetPoint(unpack(spec.defaultPoint))
            end
            f:Show()
            w.Update()
        end
    end

    function w.Open()  if not f:IsShown() then w.Toggle() end end
    function w.Close() if f:IsShown() then w.Toggle() end end

    windows[#windows + 1] = w
    return w
end

-- ── The two windows ─────────────────────────────────────────────────────────

--- The sections with anything in them, in drawing order.
---
--- Asked every redraw rather than remembered: a bag bought while the window is
--- open has slots from that moment, and the keyring answers zero until the
--- player owns one.
local function sectionsOf(list)
    local out = {}
    for _, entry in ipairs(list) do
        local total = 0
        for _, bag in ipairs(entry.bags) do
            total = total + (GetContainerNumSlots(bag) or 0)
        end
        if total > 0 then out[#out + 1] = {name = entry.name, bags = entry.bags} end
    end
    return out
end

local bags = newWindow{
    frameName   = "WoweeAllBagsFrame",
    itemPrefix  = "WoweeAllBagsItem",
    searchName  = "WoweeAllBagsSearch",
    sortName    = "WoweeAllBagsSort",
    moneyName   = "WoweeAllBagsMoney",
    sort        = true,
    money       = true,
    -- Where the bags themselves open, since this is what opens in their place:
    -- up from the bottom right, clear of the action bars.
    defaultPoint = {"BOTTOMRIGHT", UIParent, "BOTTOMRIGHT", -12, 100},
    store = function() return settings() end,
    sections = function()
        local list = {{name = "Inventory", bags = INVENTORY}}
        if showKeyring() then list[#list + 1] = {name = "Keyring", bags = KEYRING} end
        return sectionsOf(list)
    end,
}

--- The bank, in a window of its own.
---
--- Its contents are the client's to remember: it holds the twenty-eight
--- general slots and the bank bags from the last visit, so /bank shows what is
--- in there from anywhere. Only moving things needs a banker.
local bank = newWindow{
    frameName   = "WoweeBankFrame",
    itemPrefix  = "WoweeBankItem",
    searchName  = "WoweeBankSearch",
    title       = "Bank",
    -- Clear of the bag window's own corner, so the two do not open on top of
    -- each other the first time both are up.
    defaultPoint = {"BOTTOMLEFT", UIParent, "BOTTOMLEFT", 40, 120},
    store = function() return settings().bank end,
    sections = function()
        return sectionsOf({{name = "Bank", bags = BANK}})
    end,
}

--- Both, because one window's contents change when the other's do: a deposit
--- empties a carried slot and fills a bank one.
function WoweeAllBags_Update()
    for _, w in ipairs(windows) do w.Update() end
end

function WoweeAllBags_Toggle() bags.Toggle() end
function WoweeBank_Toggle()    bank.Toggle() end

-- Declared before the overrides at the end, which is where they are used.
function openOurs()  bags.Open()  end
function closeOurs() bags.Close() end

-- ── Events and the way in ───────────────────────────────────────────────────

local ev = CreateFrame("Frame")
ev:SetScript("OnEvent", function(self, event, arg1)
    if event == "BANKFRAME_OPENED" then
        -- The bank's own window, and only it. Opening the bank used to switch
        -- the bag window over to the bank's contents, which took away the view
        -- of what you were carrying at the one moment both are wanted: a
        -- deposit is a move from one to the other.
        if not separateBags() then bank.Open() end
    elseif event == "BANKFRAME_CLOSED" then
        bank.Close()
    elseif event == "WOWEE_SETTING_CHANGED" then
        -- Ticking "Separate bag windows" while these are open has to put them
        -- away: from that moment the bag keys go to FrameXML's windows, so left
        -- open these could not be closed by the key that opened them. Untick it
        -- and FrameXML's go instead, for the same reason.
        if arg1 == "separatebags" then
            if separateBags() then
                for _, w in ipairs(windows) do
                    if w.frame:IsShown() then w.frame:Hide() end
                end
            else
                -- FrameXML's own windows, hidden directly rather than through
                -- its CloseAllBags: that function calls CloseBackpack and
                -- CloseBag, which are this file's now, so asking it to shut
                -- Blizzard's bags shut ours instead.
                for i = 1, 13 do
                    local cf = _G['ContainerFrame' .. i]
                    if cf and cf.IsShown and cf:IsShown() then cf:Hide() end
                end
                if BankFrame and BankFrame.IsShown and BankFrame:IsShown() then
                    BankFrame:Hide()
                end
            end
        end
    end
    WoweeAllBags_Update()
end)
ev:RegisterEvent("BAG_UPDATE")
ev:RegisterEvent("ITEM_LOCK_CHANGED")
ev:RegisterEvent("PLAYER_MONEY")
ev:RegisterEvent("BANKFRAME_OPENED")
ev:RegisterEvent("BANKFRAME_CLOSED")
ev:RegisterEvent("PLAYERBANKSLOTS_CHANGED")
-- Interface > Bags, so its three controls take effect when they are clicked.
ev:RegisterEvent("WOWEE_SETTING_CHANGED")

--- Opening the bags opens the bag window instead.
---
--- The interface routes every way in through these: the bag bar's buttons, the
--- B key, a merchant closing. Replacing them is what makes this the bag window
--- rather than a second one beside it - before this, B gave you the four
--- ContainerFrames and this window as well, both holding the same items.
---
--- Defined after FrameXML, which is where addons load, so these are the last
--- word. The keyring keeps its own toggle: it is a section here rather than a
--- window, and the game's button for it is the natural thing to flip it with.
---
--- Each of these keeps the interface's own version and asks, at the moment it
--- is called, which window the player wants. Deciding once at load would mean
--- the setting only took effect after a reload, and the control it comes from
--- is a checkbox that should work when it is clicked.
local function replacing(original)
    return function(...)
        if separateBags() then
            if original then return original(...) end
            return
        end
        return WoweeAllBags_Toggle()
    end
end

local function replacingWith(original, ours)
    return function(...)
        if separateBags() then
            if original then return original(...) end
            return
        end
        return ours(...)
    end
end

--- Which window a container id belongs to.
---
--- The calls that name a bag have to be told apart now that there are two
--- windows, and the bank's own frame is what proves it: BankFrame_OnHide calls
--- CloseBankBagFrames, which is CloseBag(5) through CloseBag(11). Sent to the
--- carried bags - which is where every CloseBag went - walking away from a
--- bank shut the player's bags, having never opened them.
local BANK_BAG = {}
for _, id in ipairs(BANK) do BANK_BAG[id] = true end

local function windowFor(id)
    if id and BANK_BAG[id] then return bank end
    return bags
end

local function replacingBagCall(original, method)
    return function(id, ...)
        if separateBags() then
            if original then return original(id, ...) end
            return
        end
        return windowFor(id)[method]()
    end
end

ToggleBackpack = replacing(ToggleBackpack)
ToggleAllBags  = replacing(ToggleAllBags)
ToggleBag      = replacingBagCall(ToggleBag, "Toggle")
OpenBackpack   = replacingWith(OpenBackpack, openOurs)
OpenAllBags    = replacingWith(OpenAllBags, openOurs)
OpenBag        = replacingBagCall(OpenBag, "Open")
CloseBackpack  = replacingWith(CloseBackpack, closeOurs)
CloseAllBags   = replacingWith(CloseAllBags, closeOurs)
CloseBag       = replacingBagCall(CloseBag, "Close")

--- Whether the bag bar should draw its button as pressed. One window per set,
--- so a bag is open exactly when the window holding it is.
local wasBagOpen = IsBagOpen
IsBagOpen = function(id)
    if separateBags() then
        if wasBagOpen then return wasBagOpen(id) end
        return nil
    end
    return windowFor(id).frame:IsShown() and id or nil
end

--- The game's keyring button, pointed at the setting rather than at a
--- preference of this addon's own - so the checkbox in Interface > Bags and
--- this button are the same switch.
local wasToggleKeyRing = ToggleKeyRing
ToggleKeyRing = function(...)
    if separateBags() then
        if wasToggleKeyRing then return wasToggleKeyRing(...) end
        return
    end
    if WoweeSetSetting then
        WoweeSetSetting('showkeyring', not showKeyring())
    end
    openOurs()
    WoweeAllBags_Update()
end

SLASH_WOWEEALLBAGS1 = "/allbags"
SLASH_WOWEEALLBAGS2 = "/bags"
SlashCmdList["WOWEEALLBAGS"] = WoweeAllBags_Toggle

-- The bank without a banker, which is the half of it that needs no server:
-- what was left in there, from anywhere.
SLASH_WOWEEBANK1 = "/bank"
SlashCmdList["WOWEEBANK"] = WoweeBank_Toggle
