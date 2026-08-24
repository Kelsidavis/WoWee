#pragma once

/**
 * addon_lua_snippets.hpp - the Lua this client injects into the interface.
 *
 * Two of these are long enough to hide a mistake in: the options panels, which
 * build twelve categories of control out of the settings schema, and the coin
 * clearance, which reaches into MoneyFrame_Update. They lived as raw string
 * literals inside the functions that ran them, where a syntax error is not a
 * build failure - executeString simply answers false and the client carries on
 * with a warning in a log that is warning-only.
 *
 * Out here they can be handed to Lua by a test, which is the only way a
 * mistake in them is found before a player finds it.
 */

namespace wowee {
namespace addons {

/// The client's own settings, as panels in FrameXML's Interface Options.
///
/// Built from WoweeSettingList() rather than written out, so a setting added to
/// the schema appears without anyone editing Lua.
inline constexpr const char* kWoweeOptionsPanelLua = R"LUA(
local list = WoweeSettingList and WoweeSettingList()
if not list or #list == 0 then return end

local ROOT = "WoWee"

-- The panel container is 623 wide and a little under 500 tall, so two columns
-- of roughly 300 fit side by side with room for a slider's own labels. A
-- category that outgrows both columns is a category that wants splitting;
-- rather than clip it, the layout keeps going down the second column and the
-- overflow is visible, which is the version of this failure someone notices.
-- Which of the three options frames each category belongs on. The game menu
-- has a button for each, and a setting the player cannot reach from one of
-- them may as well not exist.
-- Every setting by key, so a greyed control can name the one it waits on by
-- its label rather than by its schema key.
local byKey = {}
for _, s in ipairs(list) do byKey[s.key] = s end

local kCategoryHost = {
    ["Graphics"]     = "video",
    ["Grass"]        = "video",
    ["Upscaling"]    = "video",
    ["Display"]      = "video",
    ["Sound"]        = "audio",
}

local COLUMN_X      = {16, 326}
local COLUMN_TOP    = -52
local COLUMN_BOTTOM = -436
local COLUMN_WIDTH  = 290

-- Frame names are looked up in _G, so a category's name has to survive being
-- part of one. "Combat & HUD" would not.
local function slug(text)
    return (tostring(text):gsub("[^%a%d]", ""))
end

-- A number as a person reads it: no trailing zeros, and no lone point.
local function num(v)
    if v == math.floor(v) then return tostring(math.floor(v)) end
    return (string.format("%.2f", v):gsub("0+$", ""):gsub("%.$", ""))
end

-- Whether a control is worth offering yet, from the schema's own test against
-- another setting: "" always, "key" whenever that one is on, "key=2" and
-- "key!=2" comparing its value.
--
-- The same rule the settings window applies, because it is the same field.
-- Without it these panels offered the FSR quality dropdown with upscaling off
-- and the anti-aliasing dropdown while FSR 3 was doing its own - controls that
-- answer, save, and change nothing.
-- What a greyed control is waiting for, in words.
--
-- The schema states the condition as "shadows" or "upscaling!=2", which says
-- nothing to a player looking at a slider that will not move. Reported as
-- "drop downs are greyed out" with no idea which of them were deliberate.
local function waitingOn(setting)
    local test = setting.enabledwhen
    if not test or test == "" then return nil end
    local key, want = test:match("^(.-)!=(.*)$")
    local negated = key ~= nil
    if not key then key, want = test:match("^(.-)=(.*)$") end
    if not key then key = test end
    local other = byKey[key]
    local name = other and other.label or key
    if want == nil then
        return "Available when " .. name .. " is on."
    end
    -- The value by its own label where the schema names one, so the line reads
    -- "when Upscaling is not FSR 2" rather than "is not 2".
    local choice = want
    if other and other.choices and other.choices ~= "" then
        local i = 0
        for piece in (other.choices .. "|"):gmatch("([^|]*)|") do
            if tostring(i) == want then choice = piece break end
            i = i + 1
        end
    end
    if negated then return "Available when " .. name .. " is not " .. choice .. "." end
    return "Available when " .. name .. " is " .. choice .. "."
end

local function isEnabled(setting)
    local test = setting.enabledwhen
    if not test or test == "" then return true end
    local key, want = test:match("^(.-)!=(.*)$")
    if key then return WoweeGetSetting(key) ~= want end
    key, want = test:match("^(.-)=(.*)$")
    if key then return WoweeGetSetting(key) == want end
    local value = WoweeGetSetting(test)
    return value ~= nil and value ~= "" and value ~= "0"
end

-- Greyed rather than hidden, so the panel keeps its shape and a player can see
-- both that the setting exists and what it is waiting on.
local function setEnabled(widget, label, enabled)
    if widget.Enable and widget.Disable then
        if enabled then widget:Enable() else widget:Disable() end
    end
    if not label then return end
    -- The colour a label goes back to is the one it was created with, read
    -- once: a checkbox's is white and a slider's is the gold heading colour,
    -- and picking either would be wrong for the other half of the panel.
    if not label.woweeColor then
        local r, g, b = 1, 1, 1
        if label.GetTextColor then r, g, b = label:GetTextColor() end
        label.woweeColor = {r or 1, g or 1, b or 1}
    end
    if enabled then
        label:SetTextColor(label.woweeColor[1], label.woweeColor[2], label.woweeColor[3])
    else
        label:SetTextColor(0.5, 0.5, 0.5)
    end
end

-- The tooltip with the reason under it, when there is one.
local function joinReason(tip, reason)
    if not reason then return tip end
    if not tip or tip == "" then return reason end
    return tip .. "\n" .. reason
end

-- The game's own hover text, one line per line of the schema's tooltip.
local function withTooltip(widget, title, tip)
    if not tip or tip == "" then return end
    widget:SetScript("OnEnter", function(self)
        GameTooltip:SetOwner(self, "ANCHOR_RIGHT")
        GameTooltip:SetText(title, 1, 1, 1)
        for line in tostring(tip):gmatch("[^\n]+") do
            GameTooltip:AddLine(line, nil, nil, nil, true)
        end
        GameTooltip:Show()
    end)
    widget:SetScript("OnLeave", function() GameTooltip:Hide() end)
end

-- Where the next control goes, and when to start the second column.
--
-- Measured from the panel rather than taken from COLUMN_X, because the three
-- frames that host these panels do not have containers of one width. The
-- Interface frame's is the 623 those constants describe; the Video frame's is
-- around four hundred, because a category list the same size sits inside a
-- narrower frame. Laid out at 326 regardless, the second column started past
-- the right edge and every control in it hung off the side of the panel.
--
-- Two columns only when both fit with a gap. Otherwise one, down the middle of
-- whatever there is: a column running off the edge is worse than a long one.
local function newLayout(panel)
    -- Both dimensions from the panel, because neither constant was right.
    --
    -- COLUMN_X put the second column at 326 with a width of 290, which needs a
    -- container 623 wide; the real one is 413, on the Interface frame as much
    -- as the Video one, so everything in column two hung off the right edge.
    -- Falling back to a single column then moved the overflow to the bottom:
    -- eleven graphics controls need more height than one column has, and
    -- COLUMN_BOTTOM at -436 was already past a panel 428 tall, so nothing
    -- wrapped and the tail ran off the page.
    --
    -- So: as many columns as fit at a width still worth having, and a bottom
    -- that is the panel's own.
    local width = panel:GetWidth() or 0
    local height = panel:GetHeight() or 0
    if width <= 0 then width = 413 end
    if height <= 0 then height = 428 end

    local margin, gap, minWidth = 12, 14, 170
    local columns, columnWidth = {}, 0
    local twoWide = (width - margin * 2 - gap) / 2
    if twoWide >= minWidth then
        columnWidth = math.floor(twoWide)
        columns = {margin, margin + columnWidth + gap}
    else
        columnWidth = math.max(minWidth, width - margin * 2)
        columns = {margin}
    end

    return {panel = panel, column = 1, y = COLUMN_TOP,
            columns = columns, columnWidth = columnWidth,
            bottom = -(height - 10)}
end

local function reserve(layout, height)
    if layout.y - height < layout.bottom and layout.column < #layout.columns then
        layout.column = layout.column + 1
        layout.y = COLUMN_TOP
    end
    local x, y = layout.columns[layout.column], layout.y
    layout.y = layout.y - height
    return x, y
end

local function addHeading(layout, text)
    local x, y = reserve(layout, 32)
    local label = layout.panel:CreateFontString(nil, "ARTWORK", "GameFontNormal")
    label:SetPoint("TOPLEFT", x, y - 4)
    label:SetText(text)
    local rule = layout.panel:CreateTexture(nil, "ARTWORK")
    rule:SetTexture("Interface\\Buttons\\WHITE8X8")
    rule:SetVertexColor(0.5, 0.42, 0.22, 0.7)
    rule:SetWidth(layout.columnWidth)
    rule:SetHeight(1)
    rule:SetPoint("TOPLEFT", x, y - 22)
end

-- A region the control's template was expected to declare, or nothing.
--
-- These controls inherit WotLK's option templates, which declare $parentText
-- and, on a slider, $parentLow and $parentHigh. A 1.12 interface loads no
-- OptionsPanelTemplates.xml at all, so CreateFrame finds no template, applies
-- nothing, and none of those names is ever created.
--
-- rawget, so every key answers the same way. Through _G the answer depends on
-- whether the name happens to contain a digit: the missing-global fallback
-- hands back a silent no-op for most of them and a plain nil for those, since
-- a digit means an instance rather than an API name. So sixteen of these
-- labels were quietly set on a stand-in and never appeared, while showbar2 -
-- one setting, named for the bar it shows - raised instead, and raising here
-- happens inside buildPanel, which lost the client's whole options category.
local function templateRegion(name, suffix)
    return rawget(_G, name .. suffix)
end

-- A label of our own, for where the template gave none. Anchored to the
-- control rather than to the column so it lands where the template's would
-- have, and created on the panel so it is torn down with everything else.
local function ownLabel(panel, control, font, point, relPoint, ox, oy)
    local label = panel:CreateFontString(nil, "ARTWORK", font)
    label:SetPoint(point, control, relPoint, ox, oy)
    return label
end

-- The three controls. Each answers a read function and a write function, so
-- one refresh walks all of them without caring which kind it is holding.

local function addCheckButton(layout, panel, setting, onChanged)
    local x, y = reserve(layout, 27)
    local name = panel:GetName() .. setting.key
    local button = CreateFrame("CheckButton", name, panel,
                               "InterfaceOptionsCheckButtonTemplate")
    button:SetPoint("TOPLEFT", x, y)
    local label = templateRegion(name, "Text")
                  or ownLabel(panel, button, "GameFontHighlight", "LEFT", "RIGHT", 2, 1)
    label:SetText(setting.label)
    button:SetScript("OnClick", function(self)
        WoweeSetSetting(setting.key, self:GetChecked() and "1" or "0")
        -- Ticking one of these can be what makes another control live -
        -- normal mapping gates its strength, each extra bar gates its offsets
        -- - so the rest of the panel is re-read. A click is one event, unlike
        -- a slider drag, so there is nothing to throttle here.
        if onChanged then onChanged(setting.key) end
    end)
    withTooltip(button, setting.label,
                joinReason(setting.tooltip, waitingOn(setting)))
    return {
        read = function()
            button:SetChecked(WoweeGetSetting(setting.key) == "1")
            setEnabled(button, label, isEnabled(setting))
        end,
        write = function(value) WoweeSetSetting(setting.key, value) end,
    }
end

local function addSlider(layout, panel, setting)
    local x, y = reserve(layout, 50)
    local name = panel:GetName() .. setting.key
    local slider = CreateFrame("Slider", name, panel, "OptionsSliderTemplate")
    slider:SetPoint("TOPLEFT", x + 4, y - 14)
    slider:SetWidth(layout.columnWidth - 20)
    slider:SetMinMaxValues(setting.min, setting.max)
    slider:SetValueStep(setting.step)
    local low = templateRegion(name, "Low")
                or ownLabel(panel, slider, "GameFontHighlightSmall",
                            "TOPLEFT", "BOTTOMLEFT", 0, 2)
    local high = templateRegion(name, "High")
                 or ownLabel(panel, slider, "GameFontHighlightSmall",
                             "TOPRIGHT", "BOTTOMRIGHT", 0, 2)
    low:SetText(num(setting.min))
    high:SetText(num(setting.max))
    local valueText = templateRegion(name, "Text")
                      or ownLabel(panel, slider, "GameFontHighlight",
                                  "BOTTOMLEFT", "TOPLEFT", 0, 2)

    -- The value belongs beside the name rather than under the thumb: the
    -- template has nowhere to put a moving label, and a slider whose number is
    -- only in a tooltip is a slider nobody can set to a particular value.
    local function showValue(value)
        valueText:SetText(setting.label .. ":  " .. num(value))
    end
    slider:SetScript("OnValueChanged", function(self, value)
        showValue(value)
        WoweeSetSetting(setting.key, tostring(value))
    end)
    withTooltip(slider, setting.label,
                joinReason(setting.tooltip, waitingOn(setting)))
    return {
        read = function()
            local value = tonumber(WoweeGetSetting(setting.key)) or setting.min
            slider:SetValue(value)
            showValue(value)
            setEnabled(slider, valueText, isEnabled(setting))
        end,
        write = function(value) WoweeSetSetting(setting.key, value) end,
    }
end

local function addDropdown(layout, panel, setting, onChanged)
    local x, y = reserve(layout, 50)
    local name = panel:GetName() .. setting.key

    local label = panel:CreateFontString(nil, "ARTWORK", "GameFontHighlightSmall")
    label:SetPoint("TOPLEFT", x + 2, y - 2)
    label:SetText(setting.label)

    local choices = {}
    for choice in tostring(setting.choices or ""):gmatch("[^|]+") do
        table.insert(choices, choice)
    end

    -- The dropdown template carries about sixteen units of its own inset on the
    -- left, so it is anchored back by that much to line its box up with the
    -- checkboxes above it.
    local dropdown = CreateFrame("Frame", name, panel, "UIDropDownMenuTemplate")
    dropdown:SetPoint("TOPLEFT", x - 14, y - 16)
    -- UIDropDownMenu_SetWidth swapped its arguments at 2.0: 1.12 takes
    -- (width, frame) and everything after takes (frame, width). Called the
    -- later way on a 1.12 interface, the frame arrives as a width and the
    -- number as the frame - and uidropdownmenu.lua then indexes the number,
    -- which raised there and lost this panel with it.
    if (__WoweeInterfaceVersion or 0) >= 20000 then
        UIDropDownMenu_SetWidth(dropdown, layout.columnWidth - 60)
    else
        UIDropDownMenu_SetWidth(layout.columnWidth - 60, dropdown)
    end

    local function selected()
        return math.floor(tonumber(WoweeGetSetting(setting.key)) or 0) + 1
    end
    UIDropDownMenu_Initialize(dropdown, function(self, level)
        for index, choice in ipairs(choices) do
            local info = UIDropDownMenu_CreateInfo()
            info.text = choice
            info.value = index
            info.checked = (index == selected())
            info.func = function(button)
                WoweeSetSetting(setting.key, tostring(button.value - 1))
                UIDropDownMenu_SetText(dropdown, choices[button.value])
                CloseDropDownMenus()
                -- A dropdown can change other settings - the quality preset
                -- sets nine of them - so the rest of the panel is re-read.
                -- Only dropdowns do this: a slider would do it on every frame
                -- of a drag.
                if onChanged then onChanged(setting.key) end
            end
            UIDropDownMenu_AddButton(info, level)
        end
    end)
    withTooltip(dropdown, setting.label,
                joinReason(setting.tooltip, waitingOn(setting)))
    return {
        read = function()
            UIDropDownMenu_SetText(dropdown, choices[selected()] or "")
            local enabled = isEnabled(setting)
            setEnabled(dropdown, label, enabled)
            -- A dropdown is a frame rather than a button, so it has no Enable
            -- of its own; this is what the interface's own panels call.
            if enabled then
                if UIDropDownMenu_EnableDropDown then UIDropDownMenu_EnableDropDown(dropdown) end
            else
                if UIDropDownMenu_DisableDropDown then UIDropDownMenu_DisableDropDown(dropdown) end
            end
        end,
        write = function(value) WoweeSetSetting(setting.key, value) end,
    }
end

-- One panel per category, in the order the schema first mentions each.
local order, byCategory = {}, {}
for _, setting in ipairs(list) do
    if not byCategory[setting.category] then
        byCategory[setting.category] = {}
        table.insert(order, setting.category)
    end
    table.insert(byCategory[setting.category], setting)
end

-- Every panel registered, and every heading they nest under, so both can be
-- opened once the whole list exists.
local registered = {}
local headings = {}

-- The frame a category's panel belongs to, and the container inside it.
--
-- Needed before the controls are laid out, not just before registration: the
-- layout measures the container to decide how many columns fit, and an
-- unparented panel has no width to measure.
local function hostContainerFor(category)
    local host = kCategoryHost[category]
    local frame
    if host == "video" then frame = VideoOptionsFrame
    elseif host == "audio" then frame = AudioOptionsFrame
    else frame = InterfaceOptionsFrame end
    -- .panelContainer, or the global the XML names it by. VideoOptionsFrame
    -- and AudioOptionsFrame set the field; InterfaceOptionsFrame does not, and
    -- only has InterfaceOptionsFramePanelContainer - so its panels were left
    -- unparented, sized zero, and laid out against nothing.
    local container = frame and frame.panelContainer
    if not container and frame and frame.GetName then
        container = _G[(frame:GetName() or "") .. "PanelContainer"]
    end
    return frame, container
end

local function buildPanel(category, settings)
    local panel = CreateFrame("Frame", "WoweeOptions" .. slug(category))
    -- Parented and sized before anything is laid out inside it. See
    -- hostContainerFor and newLayout.
    local _, container = hostContainerFor(category)
    if container then
        panel:SetParent(container)
        panel:ClearAllPoints()
        panel:SetAllPoints(container)
        panel:Hide()
    end
    panel.name = category
    panel.parent = ROOT

    local title = panel:CreateFontString(nil, "ARTWORK", "GameFontNormalLarge")
    title:SetPoint("TOPLEFT", 16, -16)
    title:SetText(ROOT .. ": " .. category)

    local layout = newLayout(panel)
    local controls = {}
    local heading = nil
    -- Declared before the controls because a dropdown's handler calls it, and
    -- the controls are what it walks.
    local function rereadOthers(changedKey)
        for _, control in ipairs(controls) do
            if control.key ~= changedKey then control.read() end
        end
    end
    for _, setting in ipairs(settings) do
        if setting.section ~= "" and setting.section ~= heading then
            heading = setting.section
            addHeading(layout, heading)
        end
        local control
        if setting.kind == "bool" then
            control = addCheckButton(layout, panel, setting, rereadOthers)
        elseif setting.kind == "enum" then
            control = addDropdown(layout, panel, setting, rereadOthers)
        else
            control = addSlider(layout, panel, setting)
        end
        control.key = setting.key
        table.insert(controls, control)
    end

    -- Everything applies as it is changed, so Okay has nothing left to do.
    -- Cancel does: it puts back what was there when the panel was last shown,
    -- which is what the button promises and what the old version of this panel
    -- quietly did not honour.
    local opened = {}
    panel.refresh = function()
        for _, control in ipairs(controls) do
            opened[control.key] = WoweeGetSetting(control.key)
            control.read()
        end
    end
    panel.okay = function()
        for _, control in ipairs(controls) do
            opened[control.key] = WoweeGetSetting(control.key)
        end
    end
    panel.cancel = function()
        for _, control in ipairs(controls) do
            if opened[control.key] then control.write(opened[control.key]) end
        end
        panel.refresh()
    end
    -- The game puts a Defaults button on every options panel, and this was a
    -- function that did nothing - the schema had no defaults to put back. It
    -- has now, so the button does what it says for this panel's settings and
    -- leaves every other panel's alone.
    panel.default = function()
        for _, setting in ipairs(settings) do
            WoweeSetSetting(setting.key, tostring(setting.default))
        end
        panel.refresh()
    end

    -- Where a player will actually look for it.
    --
    -- These were all registered into Interface Options' AddOns tab, which is
    -- two levels down from the game menu and is where an addon's settings go -
    -- not the client's own. Reported as the options still being missing, and
    -- fairly: pressing Video showed the game's video panel and nothing of ours.
    --
    -- So each category goes to the frame its own button opens. The graphics
    -- ones join Video, the sound ones join Sound, and the rest join the
    -- Interface list beside the game's own categories.
    -- The panel has to be a child of the frame's panel container before it is
    -- registered. OptionsList_DisplayPanel positions it with
    --
    --     local panelContainer = panel:GetParent()
    --     panel:SetPoint("TOPLEFT", panelContainer, "TOPLEFT")
    --
    -- so the parent is what decides where it lands, and AddCategory does not
    -- set one - Blizzard's own panels are declared in XML as children of
    -- $parentPanelContainer and arrive parented. Ours were created with no
    -- parent at all, so every control drew from the screen's top-left corner,
    -- over the player frame and the chat log, while the panel it belonged to
    -- stayed empty.
    -- The panel was parented and sized in buildPanel, which the layout needs.
    local host = kCategoryHost[category]
    if host == "video" and VideoOptionsFrame and OptionsFrame_AddCategory then
        OptionsFrame_AddCategory(VideoOptionsFrame, panel)
    elseif host == "audio" and AudioOptionsFrame and OptionsFrame_AddCategory then
        OptionsFrame_AddCategory(AudioOptionsFrame, panel)
    else
        InterfaceOptions_AddCategory(panel)
    end
    table.insert(registered, panel)
end

-- A heading of our own on each frame that hosts one of our categories.
--
-- Both AddCategory functions nest a panel under an existing one whose name
-- matches panel.parent, so a heading registered first collects everything
-- after it. Without one our Sound category sat directly beside the game's own
-- Sound and the list read as two of the same thing.
local headingCount = 0
local function addHostHeading(hostFrame, blurbText)
    -- Named after the frame it goes on, not after tostring() of it.
    --
    -- That put the table's address in the global name -
    -- "WoweeOptionsHeadingtable: 0x609bda79b110" - so the name was different
    -- every run, could not be typed or looked up, and is not a shape a frame
    -- name is allowed to take. Nothing referenced it, which is why it went
    -- unnoticed rather than why it was fine.
    headingCount = headingCount + 1
    local hostName = hostFrame and hostFrame.GetName and hostFrame:GetName()
    if not hostName or hostName == "" then hostName = "Host" .. headingCount end
    local heading = CreateFrame("Frame", "WoweeOptionsHeading" .. hostName)
    -- Parented and sized like any other panel. Left unparented it anchored to
    -- the screen, so this heading's title and blurb were drawn over the player
    -- frame in the top-left corner rather than inside the options frame - the
    -- same fault the category panels had, in the one place that did not get
    -- the fix.
    local container = hostFrame and hostFrame.panelContainer
    if container then
        heading:SetParent(container)
        heading:ClearAllPoints()
        heading:SetAllPoints(container)
        heading:Hide()
    end
    heading.name = ROOT
    local title = heading:CreateFontString(nil, "ARTWORK", "GameFontNormalLarge")
    title:SetPoint("TOPLEFT", 16, -16)
    title:SetText(ROOT)
    local blurb = heading:CreateFontString(nil, "ARTWORK", "GameFontHighlight")
    blurb:SetPoint("TOPLEFT", 16, -48)
    -- Inside the panel rather than 560 wide, which is wider than the 413 the
    -- container actually is.
    blurb:SetWidth(math.max(200, (container and container:GetWidth() or 413) - 32))
    blurb:SetJustifyH("LEFT")
    blurb:SetJustifyV("TOP")
    blurb:SetText(blurbText)
    heading.okay = function() end
    heading.cancel = function() end
    heading.default = function() end
    heading.refresh = function() end
    if hostFrame and OptionsFrame_AddCategory then
        OptionsFrame_AddCategory(hostFrame, heading)
    end
    table.insert(headings, heading)
    return heading
end

-- The root. It holds no controls of its own: what it is for is to say what
-- this client's own settings are, and where the six that are not here live.
-- The root panel is laid out by hand rather than generated, so the anchors
-- below carry the room each block needs as a "needs N" note. A test reads
-- those and checks nothing is placed inside anything else - which is how a
-- search box came to be drawn straight through the two blocks under it, with
-- every behavioural check still passing.
local root = CreateFrame("Frame", "WoweeOptionsRoot")
root.name = ROOT

local rootTitle = root:CreateFontString(nil, "ARTWORK", "GameFontNormalLarge")
rootTitle:SetPoint("TOPLEFT", 16, -16) -- needs 22
rootTitle:SetText(ROOT)

local blurb = root:CreateFontString(nil, "ARTWORK", "GameFontHighlight")
blurb:SetPoint("TOPLEFT", 16, -48) -- needs 42
blurb:SetWidth(560)
blurb:SetJustifyH("LEFT")
blurb:SetJustifyV("TOP")
blurb:SetText("This client's own settings, under the headings below. "
    .. "Everything takes effect as you change it; Cancel puts back what was "
    .. "there when you opened the panel.")

local elsewhere = root:CreateFontString(nil, "ARTWORK", "GameFontNormalSmall")
elsewhere:SetPoint("TOPLEFT", 16, -240) -- needs 14
elsewhere:SetText("In the game's own panels")

local elsewhereRule = root:CreateTexture(nil, "ARTWORK")
elsewhereRule:SetTexture("Interface\\Buttons\\WHITE8X8")
elsewhereRule:SetVertexColor(0.5, 0.42, 0.22, 0.7)
elsewhereRule:SetWidth(560)
elsewhereRule:SetHeight(1)
elsewhereRule:SetPoint("TOPLEFT", 16, -258) -- needs 2

local elsewhereText = root:CreateFontString(nil, "ARTWORK", "GameFontHighlightSmall")
elsewhereText:SetPoint("TOPLEFT", 16, -268) -- needs 84
elsewhereText:SetWidth(560)
elsewhereText:SetJustifyH("LEFT")
elsewhereText:SetJustifyV("TOP")
-- As data rather than only as a sentence, because the search box above reads
-- it too. Typing "view distance" used to answer "No setting matches that" on a
-- panel that says two inches lower where view distance is.
--
-- Music and ambience were on this list and should not have been: they are rows
-- in the schema, drawn on our own Sound panel. The game's controls for them
-- reach the same values through getAudioSetting, which is what stops the two
-- disagreeing - not their being left out of here.
WOWEE_SETTINGS_ELSEWHERE = {
    -- "gamma (brightness)" because it is one control with two names: the
    -- game's panel calls it gamma, this client's own window calls it
    -- brightness, and SetGamma is what both of them end up in. A player who
    -- knows it by either word finds it.
    { panel = "Video",             names = { "resolution", "view distance",
                                             "ground clutter (grass)",
                                             "gamma (brightness)" } },
    { panel = "Sound",             names = { "enable sound", "master volume", "sound effects" } },
    { panel = "Interface",         names = { "mouse look speed (sensitivity)",
                                             "the minimap clock",
                                             "friendly nameplates" } },
    { panel = "Interface, Social", names = { "chat timestamps" } },
    { panel = "Key Bindings",      names = { "every key" } },
}

local elsewhereLines = {
    "Some settings are driven by the game's own controls rather than repeated "
    .. "here, so that the two cannot disagree:", "" }
for _, group in ipairs(WOWEE_SETTINGS_ELSEWHERE) do
    elsewhereLines[#elsewhereLines + 1] =
        "|cffffd100" .. group.panel .. "|r  ..  " .. table.concat(group.names, ", ")
end
elsewhereText:SetText(table.concat(elsewhereLines, "\n"))

-- What this build is. The version comes from the client rather than being
-- written here, where it would go stale the first time a tag was cut.
local aboutTitle = root:CreateFontString(nil, "ARTWORK", "GameFontNormalSmall")
aboutTitle:SetPoint("TOPLEFT", 16, -370) -- needs 14
aboutTitle:SetText("About")

local aboutRule = root:CreateTexture(nil, "ARTWORK")
aboutRule:SetTexture("Interface\\Buttons\\WHITE8X8")
aboutRule:SetVertexColor(0.5, 0.42, 0.22, 0.7)
aboutRule:SetWidth(560)
aboutRule:SetHeight(1)
aboutRule:SetPoint("TOPLEFT", 16, -388) -- needs 2

local aboutText = root:CreateFontString(nil, "ARTWORK", "GameFontHighlightSmall")
aboutText:SetPoint("TOPLEFT", 16, -398) -- needs 42
aboutText:SetWidth(560)
aboutText:SetJustifyH("LEFT")
aboutText:SetJustifyV("TOP")
aboutText:SetText("WoWee, a World of Warcraft client\n"
    .. (WoweeVersion and WoweeVersion() or "") .. "\n\n"
    .. "Kelsi Davis  ..  |cff66b3ffgithub.com/Kelsidavis/WoWee|r")

-- Find a setting without knowing which panel it is on.
--
-- Seventy-odd settings across twelve panels is enough that a player looking
-- for one has to guess, and guessing wrong twice is how a setting comes to be
-- reported missing. Typing here lists what matches and, more to the point,
-- says which panel each one is on.
local searchTitle = root:CreateFontString(nil, "ARTWORK", "GameFontNormalSmall")
searchTitle:SetPoint("TOPLEFT", 16, -104) -- needs 14
searchTitle:SetText("Find a setting")

local searchRule = root:CreateTexture(nil, "ARTWORK")
searchRule:SetTexture("Interface\\Buttons\\WHITE8X8")
searchRule:SetVertexColor(0.5, 0.42, 0.22, 0.7)
searchRule:SetWidth(560)
searchRule:SetHeight(1)
searchRule:SetPoint("TOPLEFT", 16, -122) -- needs 2

local searchBox = CreateFrame("EditBox", "WoweeOptionsSearchBox", root, "InputBoxTemplate")
searchBox:SetPoint("TOPLEFT", 22, -130) -- needs 22
searchBox:SetWidth(280)
searchBox:SetHeight(20)
searchBox:SetAutoFocus(false)

-- Named, so what the search decided can be read back from outside - the
-- headless runner cannot enumerate a frame's regions.
local searchResults = root:CreateFontString("WoweeOptionsSearchResults",
                                           "ARTWORK", "GameFontHighlightSmall")
searchResults:SetPoint("TOPLEFT", 16, -158) -- needs 78
searchResults:SetWidth(560)
searchResults:SetJustifyH("LEFT")
searchResults:SetJustifyV("TOP")

local function lower(text)
    return tostring(text):lower()
end

local function runSearch(query)
    query = lower(query)
    if query == "" then
        searchResults:SetText("")
        return
    end
    local found, shown, byName = 0, {}, 0
    -- The heading a row sits under counts as a name for it. "Anti-aliasing" is
    -- how the setting is spelled everywhere except in this client, where the
    -- control is called Multisampling and the key has no hyphen - so the term a
    -- player actually types matched neither, and the row above the one they
    -- wanted was the one that answered.
    --
    -- An empty section continues the one above, so the heading is tracked down
    -- the list rather than read off each row: FXAA has no section of its own
    -- and is still found by searching for anti-aliasing.
    local heading = ""
    for _, setting in ipairs(list) do
        if setting.section and setting.section ~= "" then heading = setting.section end
        -- The tooltip counts too. It is the sentence saying what the setting
        -- does, it is already in the schema, and it is where the words a
        -- player reaches for live when the label is the client's word for the
        -- thing: FXAA is found by "edges" because its own description says it
        -- smooths them.
        local named = lower(setting.label):find(query, 1, true) or
                      lower(setting.key):find(query, 1, true) or
                      lower(heading):find(query, 1, true)
        local described = not named and lower(setting.tooltip or ""):find(query, 1, true)
        if named or described then
            found = found + 1
            -- Named before described, because only five are shown and they are
            -- shown in the order the schema has them - so without this a
            -- setting called what was typed could sit below five whose
            -- descriptions merely mention it, and not be shown at all. Reading
            -- the descriptions is what made that possible: "the" matched almost
            -- nothing before and matches forty-five settings now.
            local line = "|cffffd100" .. setting.label .. "|r  in  " .. setting.category
            if named then
                byName = byName + 1
                table.insert(shown, byName, line)
            else
                shown[#shown + 1] = line
            end
        end
    end
    -- The ones the game's own panels drive, from the same list the block below
    -- is built from. A player who types "view distance" is told where it is
    -- rather than that it does not exist.
    for _, group in ipairs(WOWEE_SETTINGS_ELSEWHERE or {}) do
        for _, name in ipairs(group.names) do
            if lower(name):find(query, 1, true) then
                found = found + 1
                if found <= 5 then
                    shown[#shown + 1] = "|cffffd100" .. name ..
                                        "|r  in the game's own  " .. group.panel .. "  panel"
                end
            end
        end
    end

    -- Trimmed here rather than while collecting, because the ones named after
    -- what was typed are put in front of the ones that only mention it - and
    -- which those are is not known until the walk is done.
    while #shown > 5 do table.remove(shown) end

    if found == 0 then
        searchResults:SetText("|cff909090No setting matches that.|r")
    elseif found > 5 then
        searchResults:SetText(table.concat(shown, "\n") ..
            "\n|cff909090... and " .. (found - 5) .. " more|r")
    else
        searchResults:SetText(table.concat(shown, "\n"))
    end
end

searchBox:SetScript("OnTextChanged", function(self) runSearch(self:GetText()) end)
searchBox:SetScript("OnEscapePressed", function(self) self:SetText("") self:ClearFocus() end)

root.okay = function() end
root.cancel = function() end
root.default = function() end
root.refresh = function() end
-- The Game tab, not the AddOns one. This is the client's own settings, not an
-- addon's, and the AddOns tab is two levels down from the button a player
-- presses.
InterfaceOptions_AddCategory(root)

-- The headings first: a category can only nest under a name already in that
-- frame's list.
if VideoOptionsFrame then
    addHostHeading(VideoOptionsFrame,
        "This client's own graphics settings, under the headings below. "
        .. "The game's own Resolution and Effects panels are above.")
end
if AudioOptionsFrame then
    addHostHeading(AudioOptionsFrame,
        "This client's own sound settings. The game's Sound panel above "
        .. "carries the master, music, ambience and effects volumes.")
end

-- Backwards, because a nested category is inserted directly after its parent:
-- registering in schema order would list them in the opposite one.
for i = #order, 1, -1 do
    buildPanel(order[i], byCategory[order[i]])
end

-- Open, rather than folded away behind a plus sign.
--
-- Nesting a category hides it: AddCategory marks the new parent collapsed and
-- every child hidden, which is right for the game's own sub-panels and wrong
-- here. Everything this client adds is nested, so a player who opened Video
-- looking for it would find one row reading WoWee and nothing else - the same
-- thing as missing, which is how it was reported.
--
-- This is the state the toggle leaves them in, written directly because there
-- is no button to click yet: the parents are open and the children are not
-- hidden. Clicking the toggle still folds them away afterwards.
table.insert(headings, root)
for _, heading in ipairs(headings) do
    heading.collapsed = false
end
for _, panel in ipairs(registered) do
    panel.hidden = false
end
if VideoOptionsFrame and VideoOptionsFrame.categoryFrame then
    VideoOptionsFrame.categoryFrame:update()
end
if AudioOptionsFrame and AudioOptionsFrame.categoryFrame then
    AudioOptionsFrame.categoryFrame:update()
end
if InterfaceCategoryList_Update then InterfaceCategoryList_Update() end
)LUA";

/// The five uvars the panels declare but uvarInfo never registered.
///
/// A panel control names its global in self.uvar, but uvarInfo in
/// interfaceoptionsframe.lua is the only thing that creates one. Five names had
/// no entry, so each global stayed nil - and nil is neither "1" nor "0", so
/// both arms of every test on them were dead code:
///
///   MAP_QUEST_DIFFICULTY   7 sites in WorldMapFrame; titles never coloured
///   AUTO_QUEST_PROGRESS    a quest whose progress changed was never watched
///   CONSOLIDATE_BUFFS      the consolidation box never took a buff
///   ENABLE_COLORBLIND_MODE a dozen readers including every money frame
///   WATCH_FRAME_WIDTH      worse than dead - WatchFrame_SetWidth(nil) misses
///                          the == "0" arm and forces the tracker wide
///
/// This runs after InterfaceOptionsFrame_OnLoad has already called
/// InitializeUVars, so registering the entry is not enough on its own: the
/// pass that would have seeded the global is behind us. Each one is seeded
/// here as well, and the entry is what keeps it right from the next load on.
inline constexpr const char* kMissingUVarsLua = R"LUA(
local kMissing = {
    {"MAP_QUEST_DIFFICULTY", "mapQuestDifficulty", "1", "MAP_QUEST_DIFFICULTY_TEXT"},
    {"AUTO_QUEST_PROGRESS", "autoQuestProgress", "1", "AUTO_QUEST_PROGRESS_TEXT"},
    {"CONSOLIDATE_BUFFS", "consolidateBuffs", "0", "CONSOLIDATE_BUFFS_TEXT"},
    -- No _TEXT string exists for this one, and the event field is optional.
    {"ENABLE_COLORBLIND_MODE", "colorblindMode", "0", nil},
    {"WATCH_FRAME_WIDTH", "watchFrameWidth", "0", "WATCH_FRAME_WIDTH_TEXT"},
}
if type(uvarInfo) == "table" then
    for _, e in ipairs(kMissing) do
        local uvar, cvar, default, event = e[1], e[2], e[3], e[4]
        if not uvarInfo[uvar] then
            uvarInfo[uvar] = { default = default, cvar = cvar, event = event }
        end
        local v = GetCVar(cvar)
        if v == nil or v == "" then
            v = default
        end
        _G[uvar] = v
    end
end
-- The tracker takes its width from the global at load and was handed nil.
if WatchFrame_SetWidth then
    WatchFrame_SetWidth(WATCH_FRAME_WIDTH)
end
)LUA";

/// Controls for things this client does not do, taken out of the panels.
///
/// These used to be greyed with a reason in the tooltip. A disabled row is
/// still a row: the player reads it, works out whether it matters, and skips
/// it, and a page of those is harder to use than a shorter page of settings
/// that work. So they are removed, and the reason each one cannot work is kept
/// here as a comment for whoever wonders later.
///
/// Removing a row is not hiding it. The panels stack their controls by
/// anchoring each to the one above, so hiding one alone leaves the hole it
/// occupied and everything below it stays where it was. Anything anchored to a
/// removed control is re-anchored past it first, carrying the offsets so the
/// spacing closes up.
///
/// Two whole categories go rather than their contents: every control on them
/// is for a feature this client has none of, and an empty page in the list is
/// the same puzzle as a disabled row.
/// Take the compass "N" off the minimap.
///
/// It is anchored to the centre of the minimap and lifted 67 points, which in
/// the stock layout puts it on the rim above the dial. This client draws its
/// zone name across that same band, so the tag sits on top of the text and
/// makes it unreadable.
///
/// Hidden rather than moved: the minimap here does not rotate - north is always
/// up - so the marker is telling the player something the dial already says.
/// Minimap_UpdateRotationSetting shows it again every time the rotation setting
/// is touched, so the hide is hooked onto that rather than done once.
inline constexpr const char* kMinimapNorthTagLua = R"LUA(
-- Both of them, because which one is showing depends on a CVar.
--
-- Minimap_UpdateRotationSetting shows the compass ring when rotateMinimap is 1
-- and the plain N tag when it is 0, and hides the other. Taking only the N tag
-- away left the compass - which carries its own N at the top - sitting over the
-- zone name on any config with rotation turned on, which is what was still
-- being seen after the first attempt at this.
for _, name in ipairs({ "MinimapNorthTag", "MinimapCompassTexture" }) do
local tag = _G[name]
if tag then
    if tag.Hide then tag:Hide() end
    if tag.SetAlpha then tag:SetAlpha(0) end
    -- Take the Show away rather than racing it.
    --
    -- Hiding it once and hooking Minimap_UpdateRotationSetting was not enough:
    -- this snippet runs while the interface is still coming up, so the hook
    -- either found no function to attach to yet or the tag was shown again by
    -- one of the other paths that touch it, and the N was back over the zone
    -- name by the time anyone looked. A texture that cannot be shown stays
    -- hidden whoever asks.
    tag.Show = function() end
end
end
)LUA";

inline constexpr const char* kRemovedControlsLua = R"LUA(
local kRemoved = {
    -- Sound is mixed in software at the device's own rate, with no effect
    -- chain and no voice cap: no quality tiers, no reverb, no HRTF, no
    -- hardware path, no DSPs. The output device is whichever one the system
    -- hands over and cannot be switched. Emotes have no sound of their own,
    -- and a pet is voiced as any creature is.
    "AudioOptionsSoundPanelSoundQuality",
    "AudioOptionsSoundPanelSoundChannels",
    "AudioOptionsSoundPanelReverb",
    "AudioOptionsSoundPanelHRTF",
    "AudioOptionsSoundPanelUseHardware",
    "AudioOptionsSoundPanelEnableDSPs",
    "AudioOptionsSoundPanelEmoteSounds",
    "AudioOptionsSoundPanelPetSounds",
    "AudioOptionsSoundPanelHardwareDropDown",
    -- ...and the heading they sat under, which is left standing over nothing
    -- once they go. It is the only heading on these panels that empties: the
    -- others keep at least one control, and two frames that look like headings
    -- here are not - the brightness and quality sliders carry no cvar of their
    -- own, which is not the same as carrying no setting.
    "AudioOptionsSoundPanelHardware",

    -- The window and the swapchain are the desktop's business here. Buffering
    -- follows the vertical sync setting, frames are not queued ahead, the
    -- cursor is drawn by the interface, the window is sized by the desktop and
    -- stays resizable, brightness is applied in this client's own pipeline,
    -- and the refresh rate is not ours to set.
    "VideoOptionsResolutionPanelTripleBuffer",
    "VideoOptionsResolutionPanelFixInputLag",
    "VideoOptionsResolutionPanelHardwareCursor",
    "VideoOptionsResolutionPanelMaximized",
    "VideoOptionsResolutionPanelDisableResize",
    "VideoOptionsResolutionPanelDesktopGamma",
    "VideoOptionsResolutionPanelRefreshDropDown",

    -- Effects this pipeline has no stage for. Characters are composited at the
    -- resolution their art already has, there is no full screen glow pass and
    -- no death wash, and the terrain shader has no specular term.
    "VideoOptionsEffectsPanelPlayerTexture",
    "VideoOptionsEffectsPanelFullScreenGlow",
    "VideoOptionsEffectsPanelDeathEffect",
    "VideoOptionsEffectsPanelSpecularLighting",

    -- This camera does not tilt with the ground, does not bob, does not pivot
    -- at the ground, and uses one collision rule above and below water.
    "InterfaceOptionsCameraPanelFollowTerrain",
    "InterfaceOptionsCameraPanelHeadBob",
    "InterfaceOptionsCameraPanelSmartPivot",
    "InterfaceOptionsCameraPanelWaterCollision",

    -- Click to move is a movement mode this client does not have, so neither
    -- it nor the dropdown choosing its style has anything to do. The WoW mouse
    -- setting belongs to a particular mouse's driver rather than to a client.
    "InterfaceOptionsMousePanelClickToMove",
    "InterfaceOptionsMousePanelClickMoveStyleDropDown",
    "InterfaceOptionsMousePanelWoWMouse",

    -- No tutorials are shown and the loading screen carries no tips.
    "InterfaceOptionsHelpPanelShowTutorials",
    "InterfaceOptionsHelpPanelLoadingScreenTips",
    -- The button that puts the tutorials back is removed with them: there is
    -- nothing for it to reset.
    "InterfaceOptionsHelpPanelResetTutorials",

    -- There are no arena enemy frames, so no cast bar over one.
    "InterfaceOptionsUnitFramePanelArenaEnemyCastBar",

    -- Whichever language's sound files are installed is what plays.
    "InterfaceOptionsLanguagesPanelUseEnglishAudio",

    -- No mature language filter. The real client takes its word list from the
    -- locale data; the short English one written here filtered unevenly, and
    -- what it masked it masked for a player who had not asked. Chat arrives as
    -- it was sent. The spam filter beside it is a different thing and stays -
    -- it matches the same line pasted over and over, not the words in it.
    "InterfaceOptionsSocialPanelProfanityFilter",
}

-- Whole pages, because every control on them is for a feature that is not
-- here. Voice chat is stubs throughout - IsVoiceChatAllowedByServer answers
-- false and every VoiceChat_ entry point returns nothing - and stereo 3D has
-- no second eye to render.
local kRemovedCategories = {
    "AudioOptionsVoicePanel",
    "VideoOptionsStereoPanel",
}

local removed = {}
__WoweeRemovedControlsMissing = __WoweeRemovedControlsMissing or {}
for _, name in ipairs(kRemoved) do
    local f = _G[name]
    -- Asked for by name rather than assumed. A renamed or misspelled entry
    -- removes nothing while the list goes on claiming it, which is how a list
    -- like this rots without a sound.
    if f and f.GetName and f:GetName() == name then
        removed[f] = true
    else
        table.insert(__WoweeRemovedControlsMissing, name)
    end
end

-- Re-anchor one frame past anything removed, keeping the gap it held.
--
-- Runs before the hiding, and again on every panel refresh. Repeating it is
-- harmless: once a frame points past the removed control it no longer matches,
-- so the offsets are added once and not on each pass.
local function closeGap(f)
    if not f or not f.GetNumPoints or not f.GetPoint or not f.SetPoint then return end
    local count = f:GetNumPoints()
    if not count or count == 0 then return end
    local pts, changed = {}, false
    for i = 1, count do
        local point, rel, relPoint, x, y = f:GetPoint(i)
        x, y = x or 0, y or 0
        local guard = 0
        while rel and removed[rel] and guard < 16 do
            guard = guard + 1
            local _, rel2, relPoint2, x2, y2 = rel:GetPoint(1)
            if not rel2 then break end
            -- Take the removed control's own anchor, and its offset with it,
            -- so what was below it moves up by exactly the space it held.
            rel, relPoint = rel2, relPoint2
            x, y = x + (x2 or 0), y + (y2 or 0)
            changed = true
        end
        pts[i] = { point, rel, relPoint, x, y }
    end
    if not changed then return end
    f:ClearAllPoints()
    for _, pt in ipairs(pts) do
        if pt[2] then f:SetPoint(pt[1], pt[2], pt[3], pt[4], pt[5])
        else f:SetPoint(pt[1], pt[4], pt[5]) end
    end
end

-- Anchors closing the gap cannot work out on its own, because the panel has two
-- columns and one of them is anchored to the other.
--
-- The resolution panel puts the windowed-mode checkbox to the *right* of
-- vertical sync, 164 across, which in the stock layout is clear space under the
-- UI scale slider. Vertical sync hung off the refresh-rate dropdown, so
-- removing that dropdown pulls vertical sync up a row and takes windowed mode
-- with it - into the slider, on top of it. Closing the gap is right for
-- everything under the removed control in the same column and wrong for
-- anything beside it.
--
-- Windowed mode moves into the left column instead, under vertical sync, where
-- three removed checkboxes have left exactly the room for it.
local kMoved = {
    { "VideoOptionsResolutionPanelWindowed", "TOPLEFT",
      "VideoOptionsResolutionPanelVSync", "BOTTOMLEFT", 0, -4 },
}

-- Resolved by name once, like kRemoved: a move that silently anchors nothing is
-- the same rot, and it would leave the frame stacked where it was. Reported
-- here rather than on every refresh, so the list is not repeated per pass.
local moves = {}
for _, m in ipairs(kMoved) do
    local f, rel = _G[m[1]], _G[m[3]]
    if f and rel and f.ClearAllPoints and f.SetPoint and f.GetName and rel.GetName
       and f:GetName() == m[1] and rel:GetName() == m[3] then
        table.insert(moves, { f, m[2], rel, m[4], m[5], m[6] })
    else
        table.insert(__WoweeRemovedControlsMissing, m[1])
    end
end

local function applyMoves()
    for _, m in ipairs(moves) do
        m[1]:ClearAllPoints()
        m[1]:SetPoint(m[2], m[3], m[4], m[5], m[6])
    end
end

local panels = {}
local function applyRemoval()
    for f in pairs(removed) do
        local panel = f.GetParent and f:GetParent()
        if panel then panels[panel] = true end
    end
    for panel in pairs(panels) do
        if panel.GetChildren then
            for _, child in ipairs({ panel:GetChildren() }) do
                if not removed[child] then closeGap(child) end
            end
        end
    end
    for f in pairs(removed) do
        if f.Hide then f:Hide() end
    end
    -- After the gap closing, which is what moved the frame out of place.
    applyMoves()
end

applyRemoval()

-- A page with nothing left on it is the same puzzle as a disabled row, so it
-- leaves the list. The entry is the panel itself, and the list skips anything
-- marked hidden - which is what collapsed child categories already use.
for _, name in ipairs(kRemovedCategories) do
    local panel = _G[name]
    if panel then
        panel.hidden = true
        if panel.Hide then panel:Hide() end
    end
end
for _, frameName in ipairs({ "AudioOptionsFrameCategoryFrame", "VideoOptionsFrameCategoryFrame" }) do
    local catFrame = _G[frameName]
    if catFrame and OptionsCategoryFrame_Update then
        OptionsCategoryFrame_Update(catFrame)
    end
end

-- ...and again after each panel's own refresh, which is the thing that undoes
-- it. HookScript runs after the script it hooks and hooks what the frame is
-- really holding, which neither a frame of our own watching
-- PLAYER_ENTERING_WORLD nor replacing the global handler manages.
local hooked = {}
for panel in pairs(panels) do
    if panel.HookScript and not hooked[panel] then
        hooked[panel] = true
        panel:HookScript("OnShow", applyRemoval)
        panel:HookScript("OnEvent", applyRemoval)
    end
end
)LUA";

/// Move the coin amounts off the coins, and take off the coin textures the
/// interface adds - the money bar this client draws already has them in its
/// own art, and the second set reads as letters after each number.
/// Populating a dropdown is not opening one, so it does not get the sound.
///
/// UIDropDownMenu_Initialize calls its initialize function straight away -
/// stock behaviour, not ours - and every unit frame's initializer ends in
/// UnitPopup_ShowMenu, which finishes with PlaySound("igMainMenuOpen"). The
/// player frame, four party frames and three target frames all initialize when
/// the player enters the world, so eight copies of uEscapeScreenOpen.wav land
/// inside thirty milliseconds and stack into one loud hit.
///
/// The real client makes the same calls and is silent: they happen behind a
/// loading screen with the sound system not yet up. Ours has audio running by
/// then, so the difference is audible and reads as a jump scare.
///
/// This is the half that catches the initializers driven by the world-entry
/// packet, which arrives long after the interface has loaded. The load itself
/// is covered from C by LuaEngine::setUiSoundsSuppressed, because no script of
/// ours can run early enough for that.
///
/// Hooked rather than edited into unitpopup.lua, so the interface's own files
/// stay Blizzard's. Restored through pcall, so an initializer that raises
/// cannot leave every interface sound muted for the session.
/// Ask before keeping a new interface scale, and put the old one back if
/// nobody answers.
///
/// The scale applies as the slider moves - that is the shipped behaviour, and
/// it is what makes the control usable at all. It also means a scale you
/// cannot read is applied before you can decide whether you want it, and the
/// way out of that is the options frame you have just made unreadable.
/// WidgetTree::kMaxUserScale keeps it from ever reaching that, and this is the
/// second line of defence: fifteen seconds to say keep, or it goes back.
///
/// StaticPopup already implements exactly this - StaticPopup_OnUpdate calls
/// OnCancel with the reason "timeout" when timeleft runs out - so the dialog
/// is a registration rather than a mechanism. The countdown in the text is
/// ours, because the live update in StaticPopup_OnUpdate only runs for a
/// hardcoded list of dialog names; the dialog's own OnUpdate is called for
/// everything, so the number is written from there.
///
/// Hooked, not edited into videooptionspanels.lua: the interface data is
/// extracted game content and not somewhere our changes can live.
/// Put the Gamma slider on the scale its own value is measured in.
///
/// ResolutionPanelOptions.gamma offers -0.5 to 0.5. GetGamma answers 1 for a
/// neutral screen, so the slider sat past its own maximum, and every position
/// on it sent a number GameScreen::setGamma clamps to nearly black - the
/// control was unusable in both directions at once.
///
/// This cannot go through kCVarRanges like the view distance and camera
/// distance sliders did. Those are registered with a `cvar`, and
/// BlizzardOptionsPanel_OnEvent consults GetCVarMin/GetCVarMax only for
/// controls that have one; the gamma slider carries a `label` instead and is
/// always given the table's own numbers. So the table is what has to change,
/// and it is changed here rather than in videooptionspanels.lua because the
/// interface data is extracted game content and not ours to keep edits in.
///
/// The ceiling is what setGamma can hold - brightness runs 0 to 100 and gamma
/// is that over 50. The floor is Blizzard's own 0.3, so one drag to the left
/// cannot black the screen out.
inline constexpr const char* kOptionRangeFixesLua = R"LUA(
if ResolutionPanelOptions and ResolutionPanelOptions.gamma then
    ResolutionPanelOptions.gamma.minValue = 0.3
    ResolutionPanelOptions.gamma.maxValue = 2.0
    ResolutionPanelOptions.gamma.valueStep = 0.05
end

-- Shadow Quality is chosen when the shadow map is built, which is at start-up,
-- so it takes effect on the next run. The shipped table does not say so - the
-- original client could change it live - and a setting that silently waits for
-- a restart is indistinguishable from one that does nothing. Marked the same
-- way Texture Filtering already is, so the panel puts the requirement in the
-- control's own tooltip.
if EffectsPanelOptions and EffectsPanelOptions.extShadowQuality then
    EffectsPanelOptions.extShadowQuality.gameRestart = 1
    EffectsPanelOptions.extShadowQuality.tooltipRequirement = OPTION_RESTART_REQUIREMENT
end
)LUA";

inline constexpr const char* kUiScaleConfirmLua = R"LUA(
local kRevertSeconds = 15

-- FrameXML owns this table. This snippet runs whenever the interface was
-- *asked* for rather than when it actually arrived, so a data tree with no
-- interface directory reaches here with nothing defined - and indexing a table
-- that is not there raises on the first statement, taking the two hooks below
-- it with it. Everything else here already guards; this did not.
StaticPopupDialogs = StaticPopupDialogs or {}

StaticPopupDialogs["WOWEE_CONFIRM_UI_SCALE"] = {
    text = "Keep this interface scale?",
    button1 = KEEP_THIS_CHANGE or "Keep",
    button2 = CANCEL or "Cancel",
    timeout = kRevertSeconds,
    whileDead = 1,
    -- Escape would dismiss the dialog and leave the untried scale applied,
    -- which is the state this exists to prevent.
    hideOnEscape = 0,
    OnAccept = function(self, data)
        if data then data.baseline = nil end
    end,
    OnCancel = function(self, data, reason)
        if data and data.baseline then
            SetCVar("uiscale", data.baseline)
            local slider = VideoOptionsResolutionPanelUIScaleSlider
            if slider and slider.SetDisplayValue then
                slider:SetDisplayValue(tonumber(data.baseline) or 1)
            end
        end
    end,
    OnUpdate = function(self, elapsed)
        local text = _G[self:GetName() .. "Text"]
        if text and self.timeleft then
            text:SetFormattedText("Keep this interface scale?\n\nReverting in %d seconds.",
                                  math.ceil(self.timeleft))
        end
    end,
}

-- The baseline is read when the panel is shown rather than when the slider
-- moves: a drag is many changes and only the first of them knows what the
-- scale was before any of this started.
local panel = VideoOptionsResolutionPanel
if panel and panel.HookScript then
    panel:HookScript("OnShow", function(self)
        self.woweeUiScaleBaseline = GetCVar("uiscale")
    end)
end

local okay = VideoOptionsFrameOkay
if okay and okay.HookScript then
    okay:HookScript("OnClick", function()
        local p = VideoOptionsResolutionPanel
        local baseline = p and p.woweeUiScaleBaseline
        local current = GetCVar("uiscale")
        if baseline and current and baseline ~= current then
            StaticPopup_Show("WOWEE_CONFIRM_UI_SCALE", nil, nil,
                             { baseline = baseline })
            if p then p.woweeUiScaleBaseline = current end
        end
    end)
end
)LUA";

inline constexpr const char* kDropdownInitSilenceLua = R"LUA(
local realPlaySound = PlaySound
local silent = false
PlaySound = function(...)
    if silent then return end
    return realPlaySound(...)
end

local realInitialize = UIDropDownMenu_Initialize
if realInitialize then
    UIDropDownMenu_Initialize = function(...)
        local was = silent
        silent = true
        local ok, err = pcall(realInitialize, ...)
        silent = was
        if not ok then error(err, 0) end
    end
end
)LUA";

/// Chat lines do not fade.
///
/// ChatFrameTemplate declares displayDuration="120.0", so lines faded two
/// minutes after arriving. Zero means never here. UIErrorsFrame, which
/// declares five seconds, is left alone. Applied to existing windows and to
/// any opened later.
inline constexpr const char* kChatNoFadeLua = R"LUA(
local function hold(frame)
    if frame and frame.SetTimeVisible then
        frame:SetTimeVisible(0)
        -- The real client's own switch for this, in case anything asks it
        -- rather than the duration.
        if frame.SetFading then frame:SetFading(false) end
    end
end

for i = 1, (NUM_CHAT_WINDOWS or 10) do
    hold(_G["ChatFrame" .. i])
end

-- A window opened later gets the same treatment. FCF_OpenNewWindow is what
-- makes one, and it answers the frame it made.
local realOpen = FCF_OpenNewWindow
if type(realOpen) == "function" then
    FCF_OpenNewWindow = function(...)
        local frame = realOpen(...)
        hold(frame)
        return frame
    end
end
)LUA";

inline constexpr const char* kCoinAmountClearanceLua = R"LUA(
-- Colourblind mode off, explicitly.
--
-- It is the one thing in the whole interface that writes a letter beside a
-- coin: MoneyFrame_Update's colourblind branch does SetText(gold ..
-- GOLD_AMOUNT_SYMBOL) and clears the coin pictures, where the ordinary branch
-- writes the amount alone and leaves the coins. Reported as letters next to the
-- coins in the backpack, which is that branch running.
--
-- ENABLE_COLORBLIND_MODE used to be pinned to "0" here. The reason given was
-- that no writer existed - true of the readers, but the writer was supposed to
-- be uvarInfo in interfaceoptionsframe.lua, which had no entry for it. Pinning
-- the value fixed the nil and froze the setting off: the panel's checkbox set
-- the CVar and the global never followed. The uvar is registered now, so this
-- assignment would only overwrite it on the way past.

-- Between an amount and its own coin. Re-anchoring the buttons to each other
-- as well was tried and put copper two units worse than it started: their
-- spacing is MoneyFrame_Update's own, and it is right.
local kClearance = 6

-- The interface's own denomination letters, read once. Fallbacks because a
-- money frame can be updated before globalstrings has been through, and "12"
-- with a nil beside it raises where "12g" was wanted.
local kSymbols = {
    Gold   = GOLD_AMOUNT_SYMBOL   or "g",
    Silver = SILVER_AMOUNT_SYMBOL or "s",
    Copper = COPPER_AMOUNT_SYMBOL or "c",
}

local function nudge(frameName)
    for _, coin in ipairs({"Gold", "Silver", "Copper"}) do
        local text = _G[frameName .. coin .. "ButtonText"]
        local button = _G[frameName .. coin .. "Button"]
        if text and button then
            -- The amount and its denomination, as a letter.
            --
            -- The coin picture comes off below, and an amount with nothing
            -- beside it says nothing: a training cost read as three numbers in
            -- a row with no way to tell gold from silver. The letters are the
            -- interface's own - GOLD_AMOUNT_SYMBOL and its two siblings, what
            -- the colourblind branch writes - so they are localised and this
            -- client's own windows already print costs the same way.
            --
            -- Written from the digits rather than appended, so running twice
            -- over the same frame does not leave "12gg".
            local shown = text:GetText()
            if shown then
                local bare = shown:match("^(%d+)")
                if bare then text:SetText(bare .. kSymbols[coin]) end
            end
            -- No coin of ours.
            --
            -- MoneyFrame_Update makes a texture per denomination and slices the
            -- coin out of UI-MoneyIcons for it. The money bar this client draws
            -- already carries the coins in its own art, so those three are a
            -- second set on top of the first - and small, sliced and overlapping
            -- the amounts, they read as letters after each number. Four reports
            -- of "letters next to the coins" are that.
            --
            -- The amount then wants the whole button, since nothing sits to its
            -- right any more.
            local icon = button:GetNormalTexture()
            if icon then icon:SetTexture(nil) end
            text:ClearAllPoints()
            text:SetPoint("RIGHT", button, "RIGHT", -kClearance, 0)
        end
    end
end

local original = MoneyFrame_Update
if type(original) == "function" then
    MoneyFrame_Update = function(frameName, money, ...)
        original(frameName, money, ...)
        -- The frame may be named or handed over as a table, as the original
        -- accepts both.
        local name = frameName
        if type(name) == "table" then name = name:GetName() end
        if name then nudge(name) end
    end
end
)LUA";

}  // namespace addons
}  // namespace wowee
