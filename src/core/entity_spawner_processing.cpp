#include "core/entity_spawner.hpp"
#include "game/transport_path_repository.hpp"
#include "core/coordinates.hpp"
#include "core/logger.hpp"
#include "rendering/renderer.hpp"
#include "rendering/camera_controller.hpp"
#include "rendering/animation_controller.hpp"
#include "rendering/vk_context.hpp"
#include "rendering/character_renderer.hpp"
#include "rendering/wmo_renderer.hpp"
#include "rendering/m2_renderer.hpp"
#include "audio/npc_voice_manager.hpp"
#include "pipeline/m2_loader.hpp"
#include "pipeline/wmo_loader.hpp"
#include "rendering/animation/animation_ids.hpp"
#include "pipeline/dbc_loader.hpp"
#include "pipeline/asset_manager.hpp"
#include "pipeline/dbc_layout.hpp"
#include "game/game_handler.hpp"
#include "game/game_services.hpp"
#include "game/game_utils.hpp"
#include "game/transport_manager.hpp"

#include <cmath>
#include <algorithm>
#include <cctype>
#include <sstream>
#include <cstring>
#include <limits>

namespace wowee {
namespace core {

void EntitySpawner::processAsyncCreatureResults(bool unlimited) {
    // Check completed async model loads and finalize on main thread (GPU upload + instance creation).
    // Limit GPU model uploads per tick to avoid long main-thread stalls that can starve socket updates.
    // Even in unlimited mode (load screen), keep a small cap and budget to prevent multi-second stalls.
    static constexpr int kMaxModelUploadsPerTick = 1;
    static constexpr int kMaxModelUploadsPerTickWarmup = 1;
    static constexpr float kFinalizeBudgetMs = 2.0f;
    static constexpr float kFinalizeBudgetWarmupMs = 2.0f;
    const int maxUploadsThisTick = unlimited ? kMaxModelUploadsPerTickWarmup : kMaxModelUploadsPerTick;
    const float budgetMs = unlimited ? kFinalizeBudgetWarmupMs : kFinalizeBudgetMs;
    const auto tickStart = std::chrono::steady_clock::now();
    int modelUploads = 0;

    for (auto it = asyncCreatureLoads_.begin(); it != asyncCreatureLoads_.end(); ) {
        if (std::chrono::duration<float, std::milli>(
                std::chrono::steady_clock::now() - tickStart).count() >= budgetMs) {
            break;
        }

        if (!it->future.valid() ||
            it->future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
            ++it;
            continue;
        }

        auto result = it->future.get();
        it = asyncCreatureLoads_.erase(it);
        asyncCreatureDisplayLoads_.erase(result.displayId);

        auto requested = requestedCreatureDisplayIds_.find(result.guid);
        if (requested == requestedCreatureDisplayIds_.end() ||
            requested->second != result.displayId) {
            // A VALUES update changed the display while this model was loading.
            // The replacement request is already queued; never instantiate stale
            // geometry (notably the lumberjack's log-carrying model while chopping).
            // Release the pending marker unless a replacement queue entry still
            // holds it — a stranded marker blocks the resync sweep forever.
            erasePendingGuidIfUnqueued(result.guid);
            continue;
        }

        // Failures and cache hits need no GPU work — process them even when the
        // upload budget is exhausted. Previously the budget check was above this
        // point, blocking ALL ready futures (including zero-cost ones) after a
        // single upload, which throttled creature spawn throughput during world load.
        if (result.permanent_failure) {
            nonRenderableCreatureDisplayIds_.insert(result.displayId);
            creaturePermanentFailureGuids_.insert(result.guid);
            pendingCreatureSpawnGuids_.erase(result.guid);
            creatureSpawnRetryDeadlines_.erase(result.guid);
            continue;
        }
        if (!result.valid || !result.model) {
            pendingCreatureSpawnGuids_.erase(result.guid);
            creatureSpawnRetryDeadlines_.erase(result.guid);
            continue;
        }

        // Another async result may have already uploaded this displayId while this
        // task was still running; in that case, skip duplicate GPU upload.
        if (displayIdModelCache_.find(result.displayId) != displayIdModelCache_.end()) {
            pendingCreatureSpawnGuids_.erase(result.guid);
            creatureSpawnRetryDeadlines_.erase(result.guid);
            if (!creatureInstances_.count(result.guid) &&
                !creaturePermanentFailureGuids_.count(result.guid)) {
                PendingCreatureSpawn s{};
                s.guid = result.guid;
                s.displayId = result.displayId;
                s.x = result.x;
                s.y = result.y;
                s.z = result.z;
                s.orientation = result.orientation;
                s.scale = result.scale;
                pendingCreatureSpawns_.push_back(s);
                pendingCreatureSpawnGuids_.insert(result.guid);
            }
            continue;
        }

        // Only actual GPU uploads count toward the per-tick budget.
        if (modelUploads >= maxUploadsThisTick) {
            // Re-queue this result — it needs a GPU upload but we're at budget.
            // Push a new pending spawn so it's retried next frame.
            pendingCreatureSpawnGuids_.erase(result.guid);
            creatureSpawnRetryDeadlines_.erase(result.guid);
            PendingCreatureSpawn s{};
            s.guid = result.guid;
            s.displayId = result.displayId;
            s.x = result.x; s.y = result.y; s.z = result.z;
            s.orientation = result.orientation;
            s.scale = result.scale;
            pendingCreatureSpawns_.push_back(s);
            pendingCreatureSpawnGuids_.insert(result.guid);
            continue;
        }

        // Model parsed on background thread — upload to GPU on main thread.
        auto* charRenderer = renderer_ ? renderer_->getCharacterRenderer() : nullptr;
        if (!charRenderer) {
            pendingCreatureSpawnGuids_.erase(result.guid);
            continue;
        }

        // Count upload attempts toward the frame budget even if upload fails.
        // Otherwise repeated failures can consume an unbounded amount of frame time.
        modelUploads++;

        // Upload model to GPU (must happen on main thread)
        // Use pre-decoded BLP cache to skip main-thread texture decode
        auto uploadStart = std::chrono::steady_clock::now();
        charRenderer->setPredecodedBLPCache(&result.predecodedTextures);
        if (!charRenderer->loadModel(*result.model, result.modelId)) {
            charRenderer->setPredecodedBLPCache(nullptr);
            nonRenderableCreatureDisplayIds_.insert(result.displayId);
            creaturePermanentFailureGuids_.insert(result.guid);
            pendingCreatureSpawnGuids_.erase(result.guid);
            creatureSpawnRetryDeadlines_.erase(result.guid);
            continue;
        }
        charRenderer->setPredecodedBLPCache(nullptr);
        {
            auto uploadEnd = std::chrono::steady_clock::now();
            float uploadMs = std::chrono::duration<float, std::milli>(uploadEnd - uploadStart).count();
            if (uploadMs > 100.0f) {
                LOG_WARNING("charRenderer->loadModel took ", uploadMs, "ms displayId=", result.displayId,
                            " preDecoded=", result.predecodedTextures.size());
            }
        }
        // Save remaining pre-decoded textures (display skins) for spawnOnlineCreature
        if (!result.predecodedTextures.empty()) {
            displayIdPredecodedTextures_[result.displayId] = std::move(result.predecodedTextures);
        }
        displayIdModelCache_[result.displayId] = result.modelId;
        pendingCreatureSpawnGuids_.erase(result.guid);
        creatureSpawnRetryDeadlines_.erase(result.guid);

        // Re-queue as a normal pending spawn — model is now cached, so sync spawn is fast
        // (only creates instance + applies textures, no file I/O).
        if (!creatureInstances_.count(result.guid) &&
            !creaturePermanentFailureGuids_.count(result.guid)) {
            PendingCreatureSpawn s{};
            s.guid = result.guid;
            s.displayId = result.displayId;
            s.x = result.x;
            s.y = result.y;
            s.z = result.z;
            s.orientation = result.orientation;
            s.scale = result.scale;
            pendingCreatureSpawns_.push_back(s);
            pendingCreatureSpawnGuids_.insert(result.guid);
        }
    }
}

void EntitySpawner::processAsyncNpcCompositeResults(bool unlimited) {
    auto* charRenderer = renderer_ ? renderer_->getCharacterRenderer() : nullptr;
    if (!charRenderer) return;

    // Budget: 2ms per frame to avoid stalling when many NPCs complete skin compositing
    // simultaneously. In unlimited mode (load screen), process everything without cap.
    static constexpr float kCompositeBudgetMs = 2.0f;
    auto startTime = std::chrono::steady_clock::now();

    for (auto it = asyncNpcCompositeLoads_.begin(); it != asyncNpcCompositeLoads_.end(); ) {
        if (!unlimited) {
            float elapsed = std::chrono::duration<float, std::milli>(
                std::chrono::steady_clock::now() - startTime).count();
            if (elapsed >= kCompositeBudgetMs) break;
        }
        if (!it->future.valid() ||
            it->future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
            ++it;
            continue;
        }
        auto result = it->future.get();
        it = asyncNpcCompositeLoads_.erase(it);

        const auto& info = result.info;

        // Set pre-decoded cache so texture loads skip synchronous BLP decode
        charRenderer->setPredecodedBLPCache(&result.predecodedTextures);

        // --- Apply skin to type-1 slots ---
        rendering::VkTexture* skinTex = nullptr;

        if (info.hasBakedSkin) {
            // Baked skin: load from pre-decoded cache
            skinTex = charRenderer->loadTexture(info.bakedSkinPath);
        }

        if (info.hasComposite) {
            // Composite with face/underwear/equipment regions on top of base skin
            rendering::VkTexture* compositeTex = nullptr;
            if (!info.regionLayers.empty()) {
                compositeTex = charRenderer->compositeWithRegions(info.basePath,
                    info.overlayPaths, info.regionLayers);
            } else if (!info.overlayPaths.empty()) {
                std::vector<std::string> skinLayers;
                skinLayers.push_back(info.basePath);
                for (const auto& op : info.overlayPaths) skinLayers.push_back(op);
                compositeTex = charRenderer->compositeTextures(skinLayers);
            }
            if (compositeTex) skinTex = compositeTex;
        } else if (info.hasSimpleSkin) {
            // Simple skin: just base texture, no compositing
            auto* baseTex = charRenderer->loadTexture(info.basePath);
            if (baseTex) skinTex = baseTex;
        }

        if (skinTex) {
            for (uint32_t slot : info.skinTextureSlots) {
                charRenderer->setModelTexture(info.modelId, slot, skinTex);
            }
        }

        // --- Apply hair texture to type-6 slots ---
        if (!info.hairTexturePath.empty()) {
            rendering::VkTexture* hairTex = charRenderer->loadTexture(info.hairTexturePath);
            rendering::VkTexture* whTex = charRenderer->loadTexture("");
            if (hairTex && hairTex != whTex) {
                for (uint32_t slot : info.hairTextureSlots) {
                    charRenderer->setModelTexture(info.modelId, slot, hairTex);
                }
            }
        } else if (info.useBakedForHair && skinTex) {
            // Bald NPC: use skin/baked texture for scalp cap
            for (uint32_t slot : info.hairTextureSlots) {
                charRenderer->setModelTexture(info.modelId, slot, skinTex);
            }
        }

        charRenderer->setPredecodedBLPCache(nullptr);
    }
}

void EntitySpawner::processCreatureSpawnQueue(bool unlimited) {
    auto startTime = std::chrono::steady_clock::now();
    // Budget: max 2ms per frame for creature spawning to prevent stutter.
    // In unlimited mode (load screen), process everything without budget cap.
    static constexpr float kSpawnBudgetMs = 2.0f;

    // First, finalize any async model loads that completed on background threads.
    processAsyncCreatureResults(unlimited);
    {
        auto now = std::chrono::steady_clock::now();
        float asyncMs = std::chrono::duration<float, std::milli>(now - startTime).count();
        if (asyncMs > 100.0f) {
            LOG_WARNING("processAsyncCreatureResults took ", asyncMs, "ms");
        }
    }

    if (pendingCreatureSpawns_.empty()) return;
    if (!creatureLookupsBuilt_) {
        buildCreatureDisplayLookups();
        if (!creatureLookupsBuilt_) return;
    }

    int processed = 0;
    int asyncLaunched = 0;
    size_t rotationsLeft = pendingCreatureSpawns_.size();
    while (!pendingCreatureSpawns_.empty() &&
           (unlimited || processed < MAX_SPAWNS_PER_FRAME) &&
           rotationsLeft > 0) {
        // Check time budget every iteration (including first — async results may
        // have already consumed the budget via GPU model uploads).
        if (!unlimited) {
            auto now = std::chrono::steady_clock::now();
            float elapsedMs = std::chrono::duration<float, std::milli>(now - startTime).count();
            if (elapsedMs >= kSpawnBudgetMs) break;
        }

        PendingCreatureSpawn s = pendingCreatureSpawns_.front();
        pendingCreatureSpawns_.pop_front();

        auto requested = requestedCreatureDisplayIds_.find(s.guid);
        if (requested == requestedCreatureDisplayIds_.end() ||
            requested->second != s.displayId) {
            // Stale entry superseded by a newer display request for this guid.
            erasePendingGuidIfUnqueued(s.guid);
            continue;
        }

        if (nonRenderableCreatureDisplayIds_.count(s.displayId)) {
            pendingCreatureSpawnGuids_.erase(s.guid);
            creatureSpawnRetryDeadlines_.erase(s.guid);
            processed++;
            rotationsLeft = pendingCreatureSpawns_.size();
            continue;
        }

        const bool needsNewModel = (displayIdModelCache_.find(s.displayId) == displayIdModelCache_.end());

        // For new models: launch async load on background thread instead of blocking.
        if (needsNewModel) {
            // Keep exactly one background load per displayId. Additional spawns for
            // the same displayId stay queued and will spawn once cache is populated.
            if (asyncCreatureDisplayLoads_.count(s.displayId)) {
                pendingCreatureSpawns_.push_back(s);
                rotationsLeft--;
                continue;
            }

            const int maxAsync = unlimited ? (MAX_ASYNC_CREATURE_LOADS * 4) : MAX_ASYNC_CREATURE_LOADS;
            if (static_cast<int>(asyncCreatureLoads_.size()) + asyncLaunched >= maxAsync) {
                // Too many in-flight — defer to next frame
                pendingCreatureSpawns_.push_back(s);
                rotationsLeft--;
                continue;
            }

            std::string m2Path = getModelPathForDisplayId(s.displayId);
            if (m2Path.empty()) {
                nonRenderableCreatureDisplayIds_.insert(s.displayId);
                creaturePermanentFailureGuids_.insert(s.guid);
                pendingCreatureSpawnGuids_.erase(s.guid);
                creatureSpawnRetryDeadlines_.erase(s.guid);
                processed++;
                rotationsLeft = pendingCreatureSpawns_.size();
                continue;
            }

            // Check for invisible stalkers
            {
                std::string lowerPath = m2Path;
                std::transform(lowerPath.begin(), lowerPath.end(), lowerPath.begin(),
                               [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
                if (lowerPath.find("invisiblestalker") != std::string::npos ||
                    lowerPath.find("invisible_stalker") != std::string::npos) {
                    nonRenderableCreatureDisplayIds_.insert(s.displayId);
                    creaturePermanentFailureGuids_.insert(s.guid);
                    pendingCreatureSpawnGuids_.erase(s.guid);
                    processed++;
                    rotationsLeft = pendingCreatureSpawns_.size();
                    continue;
                }
            }

            // Launch async M2 load — file I/O and parsing happen off the main thread.
            uint32_t modelId = nextCreatureModelId_++;
            auto* am = assetManager_;

            // Collect display skin texture paths for background pre-decode
            std::vector<std::string> displaySkinPaths;
            {
                auto itDD = displayDataMap_.find(s.displayId);
                if (itDD != displayDataMap_.end()) {
                    std::string modelDir;
                    size_t lastSlash = m2Path.find_last_of("\\/");
                    if (lastSlash != std::string::npos) modelDir = m2Path.substr(0, lastSlash + 1);

                    auto resolveForAsync = [&](const std::string& skinField) {
                        if (skinField.empty()) return;
                        std::string raw = skinField;
                        std::replace(raw.begin(), raw.end(), '/', '\\');
                        while (!raw.empty() && std::isspace(static_cast<unsigned char>(raw.front()))) raw.erase(raw.begin());
                        while (!raw.empty() && std::isspace(static_cast<unsigned char>(raw.back()))) raw.pop_back();
                        if (raw.empty()) return;
                        bool hasExt = raw.size() >= 4 && raw.substr(raw.size()-4) == ".blp";
                        bool hasDir = raw.find('\\') != std::string::npos;
                        std::vector<std::string> candidates;
                        if (hasDir) {
                            candidates.push_back(raw);
                            if (!hasExt) candidates.push_back(raw + ".blp");
                        } else {
                            candidates.push_back(modelDir + raw);
                            if (!hasExt) candidates.push_back(modelDir + raw + ".blp");
                            candidates.push_back(raw);
                            if (!hasExt) candidates.push_back(raw + ".blp");
                        }
                        for (const auto& c : candidates) {
                            if (am->fileExists(c)) { displaySkinPaths.push_back(c); return; }
                        }
                    };
                    resolveForAsync(itDD->second.skin1);
                    resolveForAsync(itDD->second.skin2);
                    resolveForAsync(itDD->second.skin3);

                    // Pre-decode humanoid NPC textures (bake, skin, face, underwear, hair, equipment)
                    if (itDD->second.extraDisplayId != 0) {
                        auto itHE = humanoidExtraMap_.find(itDD->second.extraDisplayId);
                        if (itHE != humanoidExtraMap_.end()) {
                            const auto& he = itHE->second;
                            // Baked texture
                            if (!he.bakeName.empty()) {
                                displaySkinPaths.push_back("Textures\\BakedNpcTextures\\" + he.bakeName);
                            }
                            // CharSections: skin, face, underwear
                            auto csDbc = am->loadDBC("CharSections.dbc");
                            if (csDbc) {
                                const auto* csL = pipeline::getActiveDBCLayout()
                                    ? pipeline::getActiveDBCLayout()->getLayout("CharSections") : nullptr;
                                auto csF = pipeline::detectCharSectionsFields(csDbc.get(), csL);
                                uint32_t nRace = static_cast<uint32_t>(he.raceId);
                                uint32_t nSex = static_cast<uint32_t>(he.sexId);
                                uint32_t nSkin = static_cast<uint32_t>(he.skinId);
                                uint32_t nFace = static_cast<uint32_t>(he.faceId);
                                for (uint32_t r = 0; r < csDbc->getRecordCount(); r++) {
                                    uint32_t rId = csDbc->getUInt32(r, csF.raceId);
                                    uint32_t sId = csDbc->getUInt32(r, csF.sexId);
                                    if (rId != nRace || sId != nSex) continue;
                                    uint32_t section = csDbc->getUInt32(r, csF.baseSection);
                                    uint32_t variation = csDbc->getUInt32(r, csF.variationIndex);
                                    uint32_t color = csDbc->getUInt32(r, csF.colorIndex);
                                    if (section == 0 && color == nSkin) {
                                        std::string t = csDbc->getString(r, csF.texture1);
                                        if (!t.empty()) displaySkinPaths.push_back(t);
                                    } else if (section == 1 && variation == nFace && color == nSkin) {
                                        std::string t1 = csDbc->getString(r, csF.texture1);
                                        std::string t2 = csDbc->getString(r, csF.texture2);
                                        if (!t1.empty()) displaySkinPaths.push_back(t1);
                                        if (!t2.empty()) displaySkinPaths.push_back(t2);
                                    } else if (section == 3 && variation == static_cast<uint32_t>(he.hairStyleId)
                                               && color == static_cast<uint32_t>(he.hairColorId)) {
                                        std::string t = csDbc->getString(r, csF.texture1);
                                        if (!t.empty()) displaySkinPaths.push_back(t);
                                    } else if (section == 4 && color == nSkin) {
                                        for (uint32_t f = csF.texture1; f <= csF.texture1 + 2; f++) {
                                            std::string t = csDbc->getString(r, f);
                                            if (!t.empty()) displaySkinPaths.push_back(t);
                                        }
                                    }
                                }
                            }
                            // Equipment region textures
                            auto idiDbc = am->loadDBC("ItemDisplayInfo.dbc");
                            if (idiDbc) {
                                static constexpr const char* compDirs[] = {
                                    "ArmUpperTexture", "ArmLowerTexture", "HandTexture",
                                    "TorsoUpperTexture", "TorsoLowerTexture",
                                    "LegUpperTexture", "LegLowerTexture", "FootTexture",
                                };
                                const auto* idiL = pipeline::getActiveDBCLayout()
                                    ? pipeline::getActiveDBCLayout()->getLayout("ItemDisplayInfo") : nullptr;
                                const uint32_t trf[8] = {
                                    idiL ? (*idiL)["TextureArmUpper"]  : 14u,
                                    idiL ? (*idiL)["TextureArmLower"]  : 15u,
                                    idiL ? (*idiL)["TextureHand"]      : 16u,
                                    idiL ? (*idiL)["TextureTorsoUpper"]: 17u,
                                    idiL ? (*idiL)["TextureTorsoLower"]: 18u,
                                    idiL ? (*idiL)["TextureLegUpper"]  : 19u,
                                    idiL ? (*idiL)["TextureLegLower"]  : 20u,
                                    idiL ? (*idiL)["TextureFoot"]      : 21u,
                                };
                                const bool isFem = (he.sexId == 1);
                                for (int eq = 0; eq < 11; eq++) {
                                    uint32_t did = he.equipDisplayId[eq];
                                    if (did == 0) continue;
                                    int32_t recIdx = idiDbc->findRecordById(did);
                                    if (recIdx < 0) continue;
                                    for (int region = 0; region < 8; region++) {
                                        std::string texName = idiDbc->getString(static_cast<uint32_t>(recIdx), trf[region]);
                                        if (texName.empty()) continue;
                                        std::string base = "Item\\TextureComponents\\" +
                                            std::string(compDirs[region]) + "\\" + texName;
                                        std::string gp = base + (isFem ? "_F.blp" : "_M.blp");
                                        std::string up = base + "_U.blp";
                                        std::string bp = base + ".blp";
                                        if (am->fileExists(gp)) displaySkinPaths.push_back(gp);
                                        else if (am->fileExists(up)) displaySkinPaths.push_back(up);
                                        else if (am->fileExists(bp)) displaySkinPaths.push_back(bp);
                                    }
                                }
                            }
                        }
                    }
                }
            }

            AsyncCreatureLoad load;
            load.future = std::async(std::launch::async,
                [am, m2Path, modelId, s, skinPaths = std::move(displaySkinPaths)]() -> PreparedCreatureModel {
                    PreparedCreatureModel result;
                    result.guid = s.guid;
                    result.displayId = s.displayId;
                    result.modelId = modelId;
                    result.x = s.x;
                    result.y = s.y;
                    result.z = s.z;
                    result.orientation = s.orientation;
                    result.scale = s.scale;

                    auto m2Data = am->readFile(m2Path);
                    if (m2Data.empty()) {
                        result.permanent_failure = true;
                        return result;
                    }

                    auto model = std::make_shared<pipeline::M2Model>(pipeline::M2Loader::load(m2Data));
                    if (model->name.empty()) model->name = m2Path;
                    if (model->vertices.empty()) {
                        result.permanent_failure = true;
                        return result;
                    }

                    // Load skin file
                    if (model->version >= 264) {
                        std::string skinPath = m2Path.substr(0, m2Path.size() - 3) + "00.skin";
                        auto skinData = am->readFile(skinPath);
                        if (!skinData.empty()) {
                            pipeline::M2Loader::loadSkin(skinData, *model);
                        }
                    }

                    // Load external .anim files
                    std::string basePath = m2Path.substr(0, m2Path.size() - 3);
                    for (uint32_t si = 0; si < model->sequences.size(); si++) {
                        if (!(model->sequences[si].flags & 0x20)) {
                            char animFileName[256];
                            snprintf(animFileName, sizeof(animFileName), "%s%04u-%02u.anim",
                                basePath.c_str(), model->sequences[si].id, model->sequences[si].variationIndex);
                            auto animData = am->readFileOptional(animFileName);
                            if (!animData.empty()) {
                                pipeline::M2Loader::loadAnimFile(m2Data, animData, si, *model);
                            }
                        }
                    }

                    // Pre-decode model textures on background thread
                    for (const auto& tex : model->textures) {
                        if (tex.filename.empty()) continue;
                        std::string texKey = tex.filename;
                        std::replace(texKey.begin(), texKey.end(), '/', '\\');
                        std::transform(texKey.begin(), texKey.end(), texKey.begin(),
                                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
                        if (result.predecodedTextures.find(texKey) != result.predecodedTextures.end()) continue;
                        auto blp = am->loadTexture(texKey);
                        if (blp.isValid()) {
                            result.predecodedTextures[texKey] = std::move(blp);
                        }
                    }

                    // Pre-decode display skin textures (skin1/skin2/skin3 from CreatureDisplayInfo)
                    for (const auto& sp : skinPaths) {
                        std::string key = sp;
                        std::replace(key.begin(), key.end(), '/', '\\');
                        std::transform(key.begin(), key.end(), key.begin(),
                                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
                        if (result.predecodedTextures.count(key)) continue;
                        auto blp = am->loadTexture(key);
                        if (blp.isValid()) {
                            result.predecodedTextures[key] = std::move(blp);
                        }
                    }

                    result.model = std::move(model);
                    result.valid = true;
                    return result;
                });
            asyncCreatureLoads_.push_back(std::move(load));
            asyncCreatureDisplayLoads_.insert(s.displayId);
            asyncLaunched++;
            // Don't erase from pendingCreatureSpawnGuids_ — the async result handler will do it
            rotationsLeft = pendingCreatureSpawns_.size();
            processed++;
            continue;
        }

        // Guard the invariant spawnOnlineCreature relies on: it will not load a model
        // itself, so a cache miss here must go back through the async path rather than
        // silently dropping the creature.
        if (displayIdModelCache_.find(s.displayId) == displayIdModelCache_.end()) {
            pendingCreatureSpawns_.push_back(s);
            rotationsLeft--;
            continue;
        }

        // Cached model — spawn is fast (no file I/O, just instance creation + texture setup)
        {
            auto spawnStart = std::chrono::steady_clock::now();
            spawnOnlineCreature(s.guid, s.displayId, s.x, s.y, s.z, s.orientation, s.scale);
            auto spawnEnd = std::chrono::steady_clock::now();
            float spawnMs = std::chrono::duration<float, std::milli>(spawnEnd - spawnStart).count();
            if (spawnMs > 100.0f) {
                LOG_WARNING("spawnOnlineCreature took ", spawnMs, "ms displayId=", s.displayId);
            }
        }
        pendingCreatureSpawnGuids_.erase(s.guid);

        // If spawn still failed, retry for a bounded wall-clock window. A frame
        // count made the timeout vary wildly with refresh rate and load stalls.
        if (!creatureInstances_.count(s.guid)) {
            if (creaturePermanentFailureGuids_.erase(s.guid) > 0) {
                creatureSpawnRetryDeadlines_.erase(s.guid);
                processed++;
                continue;
            }
            const auto now = std::chrono::steady_clock::now();
            const auto deadlineIt = creatureSpawnRetryDeadlines_.try_emplace(
                s.guid, now + CREATURE_SPAWN_RETRY_WINDOW).first;
            if (now < deadlineIt->second) {
                pendingCreatureSpawns_.push_back(s);
                pendingCreatureSpawnGuids_.insert(s.guid);
            } else {
                creatureSpawnRetryDeadlines_.erase(s.guid);
                const int used = ++creatureSpawnRetryWindowsUsed_[s.guid];
                if (used < MAX_CREATURE_SPAWN_RETRY_WINDOWS) {
                    // Another window rather than an abandonment. Nothing asks
                    // again once this queue lets go: the server does not
                    // re-send an object already in range, so the creature was
                    // simply missing from then on.
                    LOG_WARNING("Creature spawn still failing after retry window ",
                                used, " of ", MAX_CREATURE_SPAWN_RETRY_WINDOWS,
                                ": guid=0x", std::hex, s.guid, std::dec,
                                " displayId=", s.displayId, " — retrying");
                    pendingCreatureSpawns_.push_back(s);
                    pendingCreatureSpawnGuids_.insert(s.guid);
                } else {
                    creatureSpawnRetryWindowsUsed_.erase(s.guid);
                    LOG_WARNING("Dropping creature spawn after ",
                                MAX_CREATURE_SPAWN_RETRY_WINDOWS,
                                " retry windows: guid=0x",
                                std::hex, s.guid, std::dec,
                                " displayId=", s.displayId);
                }
            }
        } else {
            creatureSpawnRetryDeadlines_.erase(s.guid);
            creatureSpawnRetryWindowsUsed_.erase(s.guid);
        }
        rotationsLeft = pendingCreatureSpawns_.size();
        processed++;
    }
}

void EntitySpawner::processPlayerSpawnQueue() {
    if (pendingPlayerSpawns_.empty()) return;
    if (!assetManager_ || !assetManager_->isInitialized()) return;

    int processed = 0;
    while (!pendingPlayerSpawns_.empty() && processed < MAX_SPAWNS_PER_FRAME) {
        PendingPlayerSpawn s = pendingPlayerSpawns_.front();
        pendingPlayerSpawns_.erase(pendingPlayerSpawns_.begin());
        pendingPlayerSpawnGuids_.erase(s.guid);

        // Skip if already spawned (could have been spawned by a previous update this frame)
        if (playerInstances_.count(s.guid)) {
            processed++;
            continue;
        }

        spawnOnlinePlayer(s.guid, s.raceId, s.genderId, s.appearanceBytes, s.facialFeatures, s.x, s.y, s.z, s.orientation);
        // Apply any equipment updates that arrived before the player was spawned.
        auto pit = pendingOnlinePlayerEquipment_.find(s.guid);
        if (pit != pendingOnlinePlayerEquipment_.end()) {
            deferredEquipmentQueue_.push_back({s.guid, pit->second});
            pendingOnlinePlayerEquipment_.erase(pit);
        }
        processed++;
    }
}

std::vector<std::string> EntitySpawner::resolveEquipmentTexturePaths(uint64_t guid,
    const std::array<uint32_t, 19>& displayInfoIds,
    const std::array<uint8_t, 19>& /*inventoryTypes*/) const {
    std::vector<std::string> paths;

    auto it = onlinePlayerAppearance_.find(guid);
    if (it == onlinePlayerAppearance_.end()) return paths;
    const OnlinePlayerAppearanceState& st = it->second;

    // Add base skin + underwear paths
    if (!st.bodySkinPath.empty()) paths.push_back(st.bodySkinPath);
    for (const auto& up : st.underwearPaths) {
        if (!up.empty()) paths.push_back(up);
    }

    // Resolve equipment region texture paths (same logic as setOnlinePlayerEquipment)
    auto displayInfoDbc = assetManager_->loadDBC("ItemDisplayInfo.dbc");
    if (!displayInfoDbc) return paths;
    const auto* idiL = pipeline::getActiveDBCLayout()
        ? pipeline::getActiveDBCLayout()->getLayout("ItemDisplayInfo") : nullptr;

    static constexpr const char* componentDirs[] = {
        "ArmUpperTexture", "ArmLowerTexture", "HandTexture",
        "TorsoUpperTexture", "TorsoLowerTexture",
        "LegUpperTexture", "LegLowerTexture", "FootTexture",
    };
    uint32_t texRegionFields[8];
    pipeline::getItemDisplayInfoTextureFields(*displayInfoDbc, idiL, texRegionFields);
    const bool isFemale = (st.genderId == 1);

    for (int s = 0; s < 19; s++) {
        uint32_t did = displayInfoIds[s];
        if (did == 0) continue;
        int32_t recIdx = displayInfoDbc->findRecordById(did);
        if (recIdx < 0) continue;
        for (int region = 0; region < 8; region++) {
            std::string texName = displayInfoDbc->getString(
                static_cast<uint32_t>(recIdx), texRegionFields[region]);
            if (texName.empty()) continue;
            std::string base = "Item\\TextureComponents\\" +
                std::string(componentDirs[region]) + "\\" + texName;
            std::string genderPath = base + (isFemale ? "_F.blp" : "_M.blp");
            std::string unisexPath = base + "_U.blp";
            std::string basePath = base + ".blp";
            if (assetManager_->fileExists(genderPath)) paths.push_back(genderPath);
            else if (assetManager_->fileExists(unisexPath)) paths.push_back(unisexPath);
            else if (assetManager_->fileExists(basePath)) paths.push_back(basePath);
        }
    }
    return paths;
}

void EntitySpawner::processAsyncEquipmentResults() {
    for (auto it = asyncEquipmentLoads_.begin(); it != asyncEquipmentLoads_.end(); ) {
        if (!it->future.valid() ||
            it->future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
            ++it;
            continue;
        }
        auto result = it->future.get();
        it = asyncEquipmentLoads_.erase(it);

        auto* charRenderer = renderer_ ? renderer_->getCharacterRenderer() : nullptr;
        if (!charRenderer) continue;

        // Set pre-decoded cache so compositeWithRegions skips synchronous BLP decode
        charRenderer->setPredecodedBLPCache(&result.predecodedTextures);
        setOnlinePlayerEquipment(result.guid, result.displayInfoIds, result.inventoryTypes);
        charRenderer->setPredecodedBLPCache(nullptr);
    }
}

void EntitySpawner::processDeferredEquipmentQueue() {
    // First, finalize any completed async pre-decodes
    processAsyncEquipmentResults();

    if (deferredEquipmentQueue_.empty()) return;
    // Limit in-flight async equipment loads
    if (asyncEquipmentLoads_.size() >= 2) return;

    auto [guid, equipData] = deferredEquipmentQueue_.front();
    deferredEquipmentQueue_.erase(deferredEquipmentQueue_.begin());

    // Resolve all texture paths that compositeWithRegions will need
    auto texturePaths = resolveEquipmentTexturePaths(guid, equipData.first, equipData.second);

    if (texturePaths.empty()) {
        // No textures to pre-decode — just apply directly (fast path)
        LOG_WARNING("Equipment fast path for guid=0x", std::hex, guid, std::dec,
                    " (no textures to pre-decode)");
        setOnlinePlayerEquipment(guid, equipData.first, equipData.second);
        return;
    }
    LOG_DEBUG("Equipment async pre-decode for guid=0x", std::hex, guid, std::dec,
                " textures=", texturePaths.size());

    // Launch background BLP pre-decode
    auto* am = assetManager_;
    auto displayInfoIds = equipData.first;
    auto inventoryTypes = equipData.second;
    AsyncEquipmentLoad load;
    load.future = std::async(std::launch::async,
        [am, guid, displayInfoIds, inventoryTypes, paths = std::move(texturePaths)]() -> PreparedEquipmentUpdate {
            PreparedEquipmentUpdate result;
            result.guid = guid;
            result.displayInfoIds = displayInfoIds;
            result.inventoryTypes = inventoryTypes;
            for (const auto& path : paths) {
                std::string key = path;
                std::replace(key.begin(), key.end(), '/', '\\');
                std::transform(key.begin(), key.end(), key.begin(),
                               [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
                if (result.predecodedTextures.count(key)) continue;
                auto blp = am->loadTexture(key);
                if (blp.isValid()) {
                    result.predecodedTextures[key] = std::move(blp);
                }
            }
            return result;
        });
    asyncEquipmentLoads_.push_back(std::move(load));
}

void EntitySpawner::processAsyncGameObjectResults() {
    for (auto it = asyncGameObjectLoads_.begin(); it != asyncGameObjectLoads_.end(); ) {
        if (!it->future.valid() ||
            it->future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
            ++it;
            continue;
        }

        auto result = it->future.get();
        it = asyncGameObjectLoads_.erase(it);

        if (!result.valid || !result.isWmo || !result.wmoModel) {
            // Fallback: spawn via sync path (likely an M2 or failed WMO)
            spawnOnlineGameObject(result.guid, result.entry, result.displayId,
                                 result.x, result.y, result.z, result.orientation, result.scale);
            continue;
        }

        // WMO parsed on background thread — do GPU upload + instance creation on main thread
        auto* wmoRenderer = renderer_ ? renderer_->getWMORenderer() : nullptr;
        if (!wmoRenderer) continue;

        uint32_t modelId = 0;
        auto itCache = gameObjectDisplayIdWmoCache_.find(result.displayId);
        if (itCache == gameObjectDisplayIdWmoCache_.end()) {
            // Not uploaded yet: hand it to the incremental uploader rather than
            // pushing the whole model through here, which stalled the frame for
            // as long as 40ms on a transport. The spawn finishes in
            // finishWmoSpawn once every texture and group is up.
            PendingWmoUpload pending;
            pending.result = std::move(result);
            pending.modelId = nextGameObjectWmoModelId_++;
            pendingWmoUploads_.push_back(std::move(pending));
            continue;
        }
        modelId = itCache->second;

    finishWmoSpawn(result, modelId);
    }
}

// Creates the render instance and the transport/doodad follow-ups once a model
// is fully uploaded. Split out so both the cached path and the incremental
// uploader end the same way.
void EntitySpawner::finishWmoSpawn(const PreparedGameObjectWMO& result, uint32_t modelId) {
    auto* wmoRenderer = renderer_ ? renderer_->getWMORenderer() : nullptr;
    if (!wmoRenderer) return;

    glm::vec3 renderPos = core::coords::canonicalToRender(
        glm::vec3(result.x, result.y, result.z));
    uint32_t instanceId = wmoRenderer->createInstance(
        modelId, renderPos, glm::vec3(0.0f, 0.0f, result.orientation), result.scale);
    if (instanceId == 0) return;

    gameObjectInstances_[result.guid] = {modelId, instanceId, true};

    // The synchronous WMO path notifies TransportManager after creating the
    // render instance. Do the same here: unique/uncached transport WMOs (notably
    // the Kraken icebreaker) otherwise become visible but remain unregistered
    // and stationary forever.
    if (gameHandler_ && gameHandler_->isTransportGuid(result.guid)) {
        gameHandler_->notifyTransportSpawned(
            result.guid, result.entry, result.displayId,
            result.x, result.y, result.z, result.orientation);
    }

    // Queue transport doodad loading if applicable
    std::string lowerPath = result.modelPath;
    std::transform(lowerPath.begin(), lowerPath.end(), lowerPath.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (lowerPath.find("transport") != std::string::npos) {
        const auto* doodadTemplates = wmoRenderer->getDoodadTemplates(modelId);
        if (doodadTemplates && !doodadTemplates->empty()) {
            PendingTransportDoodadBatch batch;
            batch.guid = result.guid;
            batch.modelId = modelId;
            batch.instanceId = instanceId;
            batch.x = result.x;
            batch.y = result.y;
            batch.z = result.z;
            batch.orientation = result.orientation;
            batch.doodadBudget = doodadTemplates->size();
            pendingTransportDoodadBatches_.push_back(batch);
        }
    }
}

// Uploads one pending model per frame under a budget, finishing its spawn when
// the last texture and group are in.
void EntitySpawner::processPendingWmoUploads() {
    if (pendingWmoUploads_.empty()) return;
    auto* wmoRenderer = renderer_ ? renderer_->getWMORenderer() : nullptr;
    if (!wmoRenderer) { pendingWmoUploads_.clear(); return; }

    constexpr float kUploadBudgetMs = 6.0f;
    auto& pending = pendingWmoUploads_.front();

    wmoRenderer->setPredecodedBLPCache(&pending.result.predecodedTextures);
    const auto status = wmoRenderer->loadModelIncremental(
        *pending.result.wmoModel, pending.modelId, kUploadBudgetMs);
    wmoRenderer->setPredecodedBLPCache(nullptr);

    if (status == rendering::WMORenderer::ModelLoadResult::InProgress) return;

    if (status == rendering::WMORenderer::ModelLoadResult::Complete) {
        gameObjectDisplayIdWmoCache_[pending.result.displayId] = pending.modelId;
        finishWmoSpawn(pending.result, pending.modelId);
    } else {
        LOG_WARNING("Failed to load async gameobject WMO: ", pending.result.modelPath);
    }
    pendingWmoUploads_.erase(pendingWmoUploads_.begin());
}

void EntitySpawner::processGameObjectSpawnQueue() {
    // Finalize any completed async WMO loads first
    processAsyncGameObjectResults();
    // Then advance one in-progress upload, which may finish a spawn.
    processPendingWmoUploads();

    if (pendingGameObjectSpawns_.empty()) return;

    static int goQueueLogCounter = 0;
    if (++goQueueLogCounter % 60 == 1) {
        LOG_DEBUG("GO queue: ", pendingGameObjectSpawns_.size(), " pending, ",
                 gameObjectInstances_.size(), " spawned, ",
                 gameObjectDisplayIdFailedCache_.size(), " failed");
    }

    // Process spawns: cached WMOs and M2s go sync (cheap), uncached WMOs go async
    auto startTime = std::chrono::steady_clock::now();
    static constexpr float kBudgetMs = 2.0f;
    static constexpr int kMaxAsyncLoads = 2;

    while (!pendingGameObjectSpawns_.empty()) {
        float elapsedMs = std::chrono::duration<float, std::milli>(
            std::chrono::steady_clock::now() - startTime).count();
        if (elapsedMs >= kBudgetMs) break;

        auto& s = pendingGameObjectSpawns_.front();

        // Check if this is an uncached WMO that needs async loading
        std::string modelPath;
        if (gameObjectLookupsBuilt_) {
            // Check transport overrides first
            if (gameHandler_ && gameHandler_->isTransportGuid(s.guid)) {
                modelPath = transportModelPath(s.entry, s.displayId);
            }
            if (modelPath.empty())
                modelPath = getGameObjectModelPathForDisplayId(s.displayId);
        }

        std::string lowerPath = modelPath;
        std::transform(lowerPath.begin(), lowerPath.end(), lowerPath.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        bool isWmo = lowerPath.size() >= 4 && lowerPath.substr(lowerPath.size() - 4) == ".wmo";
        bool isCached = isWmo && gameObjectDisplayIdWmoCache_.count(s.displayId);

        if (isWmo && !isCached && !modelPath.empty() &&
            static_cast<int>(asyncGameObjectLoads_.size()) < kMaxAsyncLoads) {
            // Launch async WMO load — file I/O + parse on background thread
            auto* am = assetManager_;
            PendingGameObjectSpawn capture = s;
            std::string capturePath = modelPath;
            AsyncGameObjectLoad load;
            load.future = std::async(std::launch::async,
                [am, capture, capturePath]() -> PreparedGameObjectWMO {
                    PreparedGameObjectWMO result;
                    result.guid = capture.guid;
                    result.entry = capture.entry;
                    result.displayId = capture.displayId;
                    result.x = capture.x;
                    result.y = capture.y;
                    result.z = capture.z;
                    result.orientation = capture.orientation;
                    result.scale = capture.scale;
                    result.modelPath = capturePath;
                    result.isWmo = true;

                    auto wmoData = am->readFile(capturePath);
                    if (wmoData.empty()) return result;

                    auto wmo = std::make_shared<pipeline::WMOModel>(
                        pipeline::WMOLoader::load(wmoData));

                    // Load groups
                    if (wmo->nGroups > 0) {
                        std::string basePath = capturePath;
                        std::string ext;
                        if (basePath.size() > 4) {
                            ext = basePath.substr(basePath.size() - 4);
                            basePath = basePath.substr(0, basePath.size() - 4);
                        }
                        for (uint32_t gi = 0; gi < wmo->nGroups; gi++) {
                            char suffix[16];
                            snprintf(suffix, sizeof(suffix), "_%03u%s", gi, ext.c_str());
                            auto groupData = am->readFile(basePath + suffix);
                            if (groupData.empty()) {
                                snprintf(suffix, sizeof(suffix), "_%03u.wmo", gi);
                                groupData = am->readFile(basePath + suffix);
                            }
                            if (!groupData.empty()) {
                                pipeline::WMOLoader::loadGroup(groupData, *wmo, gi);
                            }
                        }
                    }

                    // Pre-decode WMO textures on background thread
                    for (const auto& texPath : wmo->textures) {
                        if (texPath.empty()) continue;
                        std::string texKey = texPath;
                        size_t nul = texKey.find('\0');
                        if (nul != std::string::npos) texKey.resize(nul);
                        std::replace(texKey.begin(), texKey.end(), '/', '\\');
                        std::transform(texKey.begin(), texKey.end(), texKey.begin(),
                                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
                        if (texKey.empty()) continue;
                        // Convert to .blp extension
                        if (texKey.size() >= 4) {
                            std::string ext = texKey.substr(texKey.size() - 4);
                            if (ext == ".tga" || ext == ".dds") {
                                texKey = texKey.substr(0, texKey.size() - 4) + ".blp";
                            }
                        }
                        if (result.predecodedTextures.find(texKey) != result.predecodedTextures.end()) continue;
                        auto blp = am->loadTexture(texKey);
                        if (blp.isValid()) {
                            result.predecodedTextures[texKey] = std::move(blp);
                        }
                    }

                    result.wmoModel = wmo;
                    result.valid = true;
                    return result;
                });
            asyncGameObjectLoads_.push_back(std::move(load));
            pendingGameObjectSpawns_.erase(pendingGameObjectSpawns_.begin());
            continue;
        }

        // An uncached WMO that could not get an async slot must wait for one.
        // Falling through to the synchronous path here meant decoding its
        // textures on the main thread: measured at 35-51ms for a transport,
        // against this loop's 2ms budget. The async path pre-decodes them on a
        // worker, so waiting a frame for a free slot is far cheaper than doing
        // the work here. Only reachable when several uncached WMOs arrive at
        // once — a zone with a few ships in view does exactly that.
        if (isWmo && !isCached && !modelPath.empty()) {
            break;  // retry next frame, keeping queue order
        }

        // Cached WMO or M2 — spawn synchronously (cheap)
        spawnOnlineGameObject(s.guid, s.entry, s.displayId, s.x, s.y, s.z, s.orientation, s.scale);
        pendingGameObjectSpawns_.erase(pendingGameObjectSpawns_.begin());
    }
}

void EntitySpawner::processPendingTransportRegistrations() {
    if (pendingTransportRegistrations_.empty()) return;
    if (!gameHandler_ || !renderer_) return;

    auto* transportManager = gameHandler_->getTransportManager();
    if (!transportManager) return;

    auto startTime = std::chrono::steady_clock::now();
    static constexpr int kMaxRegistrationsPerFrame = 2;
    static constexpr float kRegistrationBudgetMs = 2.0f;
    int processed = 0;

    for (auto it = pendingTransportRegistrations_.begin();
         it != pendingTransportRegistrations_.end() && processed < kMaxRegistrationsPerFrame;) {
        float elapsedMs = std::chrono::duration<float, std::milli>(
            std::chrono::steady_clock::now() - startTime).count();
        if (elapsedMs >= kRegistrationBudgetMs) break;

        const PendingTransportRegistration pending = *it;
        auto goIt = gameObjectInstances_.find(pending.guid);
        if (goIt == gameObjectInstances_.end()) {
            // GO model spawns are budgeted per frame and often land AFTER the
            // transport registration is queued. Erasing on the first miss
            // silently raced the spawn queue and left transports (Deeprun
            // Tram cars) permanently unregistered. Retry until the instance
            // exists; give up loudly after ~30s (despawned/failed model).
            if (++it->retryFrames > 1800) {
                LOG_WARNING("Transport registration dropped: GO render instance never "
                            "spawned for GUID 0x", std::hex, pending.guid, std::dec,
                            " entry=", pending.entry, " displayId=", pending.displayId);
                it = pendingTransportRegistrations_.erase(it);
            } else {
                ++it;
            }
            continue;
        }

        if (transportManager->getTransport(pending.guid)) {
            transportManager->rebindTransportInstance(
                pending.guid, goIt->second.instanceId, !goIt->second.isWmo, pending.displayId);
            transportManager->updateServerTransport(
                pending.guid, glm::vec3(pending.x, pending.y, pending.z), pending.orientation);
            it = pendingTransportRegistrations_.erase(it);
            continue;
        }

        const uint32_t wmoInstanceId = goIt->second.instanceId;
        LOG_DEBUG("Registering server transport: GUID=0x", std::hex, pending.guid, std::dec,
                 " entry=", pending.entry, " displayId=", pending.displayId, " wmoInstance=", wmoInstanceId,
                 " pos=(", pending.x, ", ", pending.y, ", ", pending.z, ")");

        // TransportAnimation.dbc is indexed by GameObject entry.
        uint32_t pathId = pending.entry;
        const bool preferServerData = gameHandler_->hasServerTransportUpdate(pending.guid);

        bool clientAnim = transportManager->isClientSideAnimation();
        LOG_DEBUG("Transport spawn callback: clientAnimation=", clientAnim,
                 " guid=0x", std::hex, pending.guid, std::dec,
                 " entry=", pending.entry, " pathId=", pathId,
                 " preferServer=", preferServerData);

        glm::vec3 canonicalSpawnPos(pending.x, pending.y, pending.z);
        // Elevators were in this list too — 807 and 808 are Gnomeregan's
        // lifts, 2454 the Searing Gorge scaffold cars, 1587 a GO named
        // "Elevator" — and being taken for a ship means the stricter
        // "must travel 25 units" check rejects their short vertical path.
        const bool shipOrZeppelinDisplay =
            game::isVehicleTransportDisplay(pending.displayId);
        bool hasUsablePath = transportManager->hasPathForEntry(pending.entry);
        if (shipOrZeppelinDisplay) {
            hasUsablePath = transportManager->hasUsableMovingPathForEntry(pending.entry, 25.0f);
        }

        LOG_DEBUG("Transport path check: entry=", pending.entry, " hasUsablePath=", hasUsablePath,
                 " preferServerData=", preferServerData, " shipOrZepDisplay=", shipOrZeppelinDisplay);

        if (preferServerData) {
            if (!hasUsablePath) {
                std::vector<glm::vec3> path = { canonicalSpawnPos };
                transportManager->loadPathFromNodes(pathId, path, false, 0.0f);
                LOG_DEBUG("Server-first strict registration: stationary fallback for GUID 0x",
                         std::hex, pending.guid, std::dec, " entry=", pending.entry);
            } else {
                LOG_DEBUG("Server-first transport registration: using entry DBC path for entry ", pending.entry);
            }
        } else if (!hasUsablePath) {
            bool allowZOnly = (pending.displayId == 455 || pending.displayId == 462);
            uint32_t inferredPath = transportManager->inferDbcPathForSpawn(
                canonicalSpawnPos, 1200.0f, allowZOnly);
            if (inferredPath != 0) {
                pathId = inferredPath;
                LOG_WARNING("Using inferred transport path ", pathId, " for entry ", pending.entry);
            } else {
                uint32_t remappedPath = transportManager->pickFallbackMovingPath(pending.entry, pending.displayId);
                if (remappedPath != 0) {
                    pathId = remappedPath;
                    LOG_WARNING("Using remapped fallback transport path ", pathId,
                             " for entry ", pending.entry, " displayId=", pending.displayId,
                             " (usableEntryPath=", transportManager->hasPathForEntry(pending.entry), ")");
                } else {
                    // DEBUG, not WARNING: TaxiPath-driven ships (Auberdine/Stormwind boats,
                    // etc.) legitimately have no TransportAnimation.dbc entry and hit this
                    // spawn-time fallback, then receive their real route via
                    // assignTaxiPathToTransport and sail normally — so this fired for boats
                    // that move fine and was misleading noise when scanning the log.
                    LOG_DEBUG("No TransportAnimation.dbc path for entry ", pending.entry,
                              " - transport will be stationary until a route is assigned");
                    std::vector<glm::vec3> path = { canonicalSpawnPos };
                    transportManager->loadPathFromNodes(pathId, path, false, 0.0f);
                }
            }
        } else {
            LOG_DEBUG("Using real transport path from TransportAnimation.dbc for entry ", pending.entry);
        }

        const bool isM2Transport = !goIt->second.isWmo;

        transportManager->registerTransport(pending.guid,
                                            wmoInstanceId,
                                            pathId,
                                            canonicalSpawnPos,
                                            pending.entry,
                                            pending.displayId,
                                            isM2Transport,
                                            pending.orientation);

        transportManager->updateServerTransport(
            pending.guid, glm::vec3(pending.x, pending.y, pending.z), pending.orientation);

        auto moveIt = pendingTransportMoves_.find(pending.guid);
        if (moveIt != pendingTransportMoves_.end()) {
            const PendingTransportMove latestMove = moveIt->second;
            transportManager->updateServerTransport(
                pending.guid, glm::vec3(latestMove.x, latestMove.y, latestMove.z), latestMove.orientation);
            LOG_DEBUG("Replayed queued transport move for GUID=0x", std::hex, pending.guid, std::dec,
                     " pos=(", latestMove.x, ", ", latestMove.y, ", ", latestMove.z,
                     ") orientation=", latestMove.orientation);
            pendingTransportMoves_.erase(moveIt);
        }

        // MO_TRANSPORT (type 15) boats route via their taxi path (data[0] ->
        // TaxiPathNode.dbc), which is a world-coordinate path and thus independent of
        // where the boat spawned. Assign it whenever the GO template is already cached
        // — not only for origin-spawned transports. Boats spawn at their dock (a
        // non-origin position), so the old origin gate here meant the cached path was
        // never applied and the boat fell back to an unrelated route. If the template
        // isn't cached yet, the GO-query response hook assigns it when it arrives.
        {
            auto goData = gameHandler_->getCachedGameObjectInfo(pending.entry);
            if (goData && goData->type == 15 && goData->hasData && goData->data[0] != 0) {
                uint32_t taxiPathId = goData->data[0];
                const uint32_t mapId = gameHandler_->getCurrentMapId();
                if (transportManager->hasTaxiPathForMap(taxiPathId, mapId)) {
                    transportManager->assignTaxiPathToTransport(pending.entry, taxiPathId, mapId);
                    LOG_DEBUG("Assigned cached TaxiPathNode path for MO_TRANSPORT entry=", pending.entry,
                             " taxiPathId=", taxiPathId, " map=", mapId);
                }
            }
        }

        if (auto* tr = transportManager->getTransport(pending.guid); tr) {
            if (pending.displayId == 3831u) {
                LOG_DEBUG("Deeprun tram registration complete: guid=0x", std::hex, pending.guid, std::dec,
                            " entry=", pending.entry,
                            " displayId=", pending.displayId,
                            " pathId=", tr->pathId,
                            " isM2=", tr->isM2,
                            " mode=", (tr->useClientAnimation ? "client" : "server"),
                            " serverUpdates=", tr->serverUpdateCount);
            } else {
                LOG_DEBUG("Transport registered: guid=0x", std::hex, pending.guid, std::dec,
                         " entry=", pending.entry, " displayId=", pending.displayId,
                         " pathId=", tr->pathId,
                         " mode=", (tr->useClientAnimation ? "client" : "server"),
                         " serverUpdates=", tr->serverUpdateCount);
            }

            glm::vec3 restoredWorldPosition(0.0f);
            if (gameHandler_->completePlayerTransportWorldTransfer(
                    pending.guid, restoredWorldPosition)) {
                const glm::vec3 renderPosition =
                    core::coords::canonicalToRender(restoredWorldPosition);
                renderer_->getCharacterPosition() = renderPosition;
                if (auto* camera = renderer_->getCameraController()) {
                    camera->teleportTo(renderPosition);
                    camera->clearMovementInputs();
                    camera->suspendGravityFor(2.0f);
                    if (auto* followTarget = camera->getFollowTargetMutable()) {
                        *followTarget = renderPosition;
                    }
                }
            }
        } else {
            LOG_DEBUG("Transport registered: guid=0x", std::hex, pending.guid, std::dec,
                     " entry=", pending.entry, " displayId=", pending.displayId,
                     " (TransportManager instance missing)");
        }

        ++processed;
        it = pendingTransportRegistrations_.erase(it);
    }
}

void EntitySpawner::processPendingTransportDoodads() {
    if (pendingTransportDoodadBatches_.empty()) return;
    if (!renderer_ || !assetManager_) return;

    auto* wmoRenderer = renderer_->getWMORenderer();
    auto* m2Renderer = renderer_->getM2Renderer();
    if (!wmoRenderer || !m2Renderer) return;

    auto startTime = std::chrono::steady_clock::now();
    static constexpr float kDoodadBudgetMs = 4.0f;

    // Batch all GPU uploads into a single async command buffer submission so that
    // N doodads with multiple textures each don't each block on vkQueueSubmit +
    // vkWaitForFences. Without batching, 30+ doodads × several textures = hundreds
    // of sync GPU submits → the 490ms stall that preceded the VK_ERROR_DEVICE_LOST.
    auto* vkCtx = renderer_->getVkContext();
    if (vkCtx) vkCtx->beginUploadBatch();

    size_t budgetLeft = MAX_TRANSPORT_DOODADS_PER_FRAME;
    for (auto it = pendingTransportDoodadBatches_.begin();
         it != pendingTransportDoodadBatches_.end() && budgetLeft > 0;) {
        // Time budget check
        float elapsedMs = std::chrono::duration<float, std::milli>(
            std::chrono::steady_clock::now() - startTime).count();
        if (elapsedMs >= kDoodadBudgetMs) break;
        auto goIt = gameObjectInstances_.find(it->guid);
        if (goIt == gameObjectInstances_.end() || !goIt->second.isWmo ||
            goIt->second.instanceId != it->instanceId || goIt->second.modelId != it->modelId) {
            it = pendingTransportDoodadBatches_.erase(it);
            continue;
        }

        const auto* doodadTemplates = wmoRenderer->getDoodadTemplates(it->modelId);
        if (!doodadTemplates || doodadTemplates->empty()) {
            it = pendingTransportDoodadBatches_.erase(it);
            continue;
        }

        const size_t maxIndex = std::min(it->doodadBudget, doodadTemplates->size());
        while (it->nextIndex < maxIndex && budgetLeft > 0) {
            // Per-doodad time budget (each does synchronous file I/O + parse + GPU upload)
            float innerMs = std::chrono::duration<float, std::milli>(
                std::chrono::steady_clock::now() - startTime).count();
            if (innerMs >= kDoodadBudgetMs) { budgetLeft = 0; break; }

            const auto& doodadTemplate = (*doodadTemplates)[it->nextIndex];
            it->nextIndex++;
            budgetLeft--;

            // Every failure below used to be a silent continue, so a transport
            // that lost a doodad lost it without a trace — the ship rendered,
            // minus its sails or its paddlewheel, and nothing said why. A v264
            // model carries no indices of its own, so a .skin that does not
            // resolve leaves isValid() false and drops the piece; that is the
            // one worth naming loudly.
            uint32_t doodadModelId = static_cast<uint32_t>(std::hash<std::string>{}(doodadTemplate.m2Path));
            auto m2Data = assetManager_->readFile(doodadTemplate.m2Path);
            if (m2Data.empty()) {
                LOG_WARNING("Transport doodad missing: ", doodadTemplate.m2Path);
                continue;
            }

            pipeline::M2Model m2Model = pipeline::M2Loader::load(m2Data);
            if (m2Model.name.empty()) m2Model.name = doodadTemplate.m2Path;
            std::string skinPath = doodadTemplate.m2Path.substr(0, doodadTemplate.m2Path.size() - 3) + "00.skin";
            std::vector<uint8_t> skinData = assetManager_->readFile(skinPath);
            if (!skinData.empty() && m2Model.version >= 264) {
                pipeline::M2Loader::loadSkin(skinData, m2Model);
            } else if (skinData.empty() && m2Model.version >= 264) {
                LOG_WARNING("Transport doodad ", doodadTemplate.m2Path, " is version ",
                            m2Model.version, " and its skin '", skinPath,
                            "' did not resolve — it has no indices and will be skipped");
            }
            if (!m2Model.isValid()) {
                LOG_WARNING("Transport doodad unusable: ", doodadTemplate.m2Path,
                            " version=", m2Model.version,
                            " verts=", m2Model.vertices.size(),
                            " indices=", m2Model.indices.size());
                continue;
            }

            if (!m2Renderer->loadModel(m2Model, doodadModelId)) {
                LOG_WARNING("Transport doodad failed to upload: ", doodadTemplate.m2Path);
                continue;
            }
            // Created at the origin and moved into place by the parent WMO's
            // transform, so its position here is a placeholder and must not be
            // deduplicated against — otherwise the second ship of a class is
            // handed the first ship's sails rather than getting its own, and
            // the thirteen barrels in a hold collapse into one barrel.
            uint32_t m2InstanceId = m2Renderer->createInstance(
                doodadModelId, glm::vec3(0.0f), glm::vec3(0.0f), 1.0f,
                /*allowPositionDedup=*/false);
            if (m2InstanceId == 0) {
                LOG_WARNING("Transport doodad got no instance: ", doodadTemplate.m2Path);
                continue;
            }
            m2Renderer->setSkipCollision(m2InstanceId, true);
            // Ship WMO children use the dedicated transport animation states:
            // 162=ShipStart, 163=ShipMoving, 164=ShipStop. Leaving them on the
            // first sequence freezes the icebreaker paddle (its sequence 0 is
            // static) and can leave the Bravery's sail rig in its furled pose.
            m2Renderer->setInstanceAnimation(m2InstanceId, 163u, true);
            std::string doodadPathLower = doodadTemplate.m2Path;
            std::transform(doodadPathLower.begin(), doodadPathLower.end(), doodadPathLower.begin(),
                [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            bool logMachinery = false;
            if (doodadPathLower.find("transportship_sails") != std::string::npos ||
                doodadPathLower.find("icebreaker_paddlewheel") != std::string::npos) {
                logMachinery = true;
            }

            wmoRenderer->addDoodadToInstance(it->instanceId, m2InstanceId, doodadTemplate.localTransform);
            if (logMachinery) {
                // Report where it actually ended up. "Spawned" only ever meant the
                // model parsed; it said nothing about whether the thing landed on
                // the ship or at the world origin.
                glm::vec3 where(0.0f);
                float radius = 0.0f;
                const bool haveBounds = m2Renderer->getInstanceBounds(m2InstanceId, where, radius);
                LOG_WARNING("Transport machinery spawned: ", doodadTemplate.m2Path,
                            " instance=", m2InstanceId,
                            " wmoInstance=", it->instanceId,
                            " hasShipMoving=", m2Renderer->hasAnimation(m2InstanceId, 163u),
                            " bounds=", haveBounds,
                            " worldPos=(", where.x, ",", where.y, ",", where.z, ")",
                            " radius=", radius);
            }
            it->spawnedDoodads++;
        }

        if (it->nextIndex >= maxIndex) {
            if (it->spawnedDoodads > 0) {
                LOG_DEBUG("Spawned ", it->spawnedDoodads,
                         " transport doodads for WMO instance ", it->instanceId);
                glm::vec3 renderPos = core::coords::canonicalToRender(glm::vec3(it->x, it->y, it->z));
                glm::mat4 wmoTransform(1.0f);
                wmoTransform = glm::translate(wmoTransform, renderPos);
                wmoTransform = glm::rotate(wmoTransform, it->orientation, glm::vec3(0, 0, 1));
                wmoRenderer->setInstanceTransform(it->instanceId, wmoTransform);
            }
            it = pendingTransportDoodadBatches_.erase(it);
        } else {
            ++it;
        }
    }

    // Finalize the upload batch — submit all GPU copies in one shot (async, no wait).
    if (vkCtx) vkCtx->endUploadBatch();
}

void EntitySpawner::processPendingMount() {
    if (pendingMountDisplayId_ == 0) return;
    uint32_t mountDisplayId = pendingMountDisplayId_;
    pendingMountDisplayId_ = 0;
    LOG_INFO("processPendingMount: loading displayId ", mountDisplayId);

    if (!renderer_ || !renderer_->getCharacterRenderer() || !assetManager_) return;
    auto* charRenderer = renderer_->getCharacterRenderer();

    std::string m2Path = getModelPathForDisplayId(mountDisplayId);
    if (m2Path.empty()) {
        LOG_WARNING("No model path for mount displayId ", mountDisplayId);
        return;
    }

    // Check model cache
    uint32_t modelId = 0;
    auto cacheIt = displayIdModelCache_.find(mountDisplayId);
    if (cacheIt != displayIdModelCache_.end()) {
        modelId = cacheIt->second;
    } else {
        modelId = nextCreatureModelId_++;

        auto m2Data = assetManager_->readFile(m2Path);
        if (m2Data.empty()) {
            LOG_WARNING("Failed to read mount M2: ", m2Path);
            return;
        }

        pipeline::M2Model model = pipeline::M2Loader::load(m2Data);
        if (model.name.empty()) model.name = m2Path;
        if (model.vertices.empty()) {
            LOG_WARNING("Failed to parse mount M2: ", m2Path);
            return;
        }

        // Load skin file (only for WotLK M2s - vanilla has embedded skin)
        if (model.version >= 264) {
            std::string skinPath = m2Path.substr(0, m2Path.size() - 3) + "00.skin";
            auto skinData = assetManager_->readFile(skinPath);
            if (!skinData.empty()) {
                pipeline::M2Loader::loadSkin(skinData, model);
            } else {
                LOG_WARNING("Missing skin file for WotLK mount M2: ", skinPath);
            }
        }

        // Load external .anim files (only idle + run needed for mounts)
        std::string basePath = m2Path.substr(0, m2Path.size() - 3);
        for (uint32_t si = 0; si < model.sequences.size(); si++) {
            if (!(model.sequences[si].flags & 0x20)) {
                uint32_t animId = model.sequences[si].id;
                // Only load stand, walk, run anims to avoid hang
                if (animId != rendering::anim::STAND && animId != rendering::anim::WALK && animId != rendering::anim::RUN) continue;
                char animFileName[256];
                snprintf(animFileName, sizeof(animFileName), "%s%04u-%02u.anim",
                    basePath.c_str(), animId, model.sequences[si].variationIndex);
                auto animData = assetManager_->readFileOptional(animFileName);
                if (!animData.empty()) {
                    pipeline::M2Loader::loadAnimFile(m2Data, animData, si, model);
                }
            }
        }

        if (!charRenderer->loadModel(model, modelId)) {
            LOG_WARNING("Failed to load mount model: ", m2Path);
            return;
        }

        displayIdModelCache_[mountDisplayId] = modelId;
    }

    // Apply creature skin textures from CreatureDisplayInfo.dbc.
    // Re-apply even for cached models so transient failures can self-heal.
    std::string modelDir;
    size_t lastSlash = m2Path.find_last_of("\\/");
    if (lastSlash != std::string::npos) {
        modelDir = m2Path.substr(0, lastSlash + 1);
    }

    auto itDisplayData = displayDataMap_.find(mountDisplayId);
    bool haveDisplayData = false;
    CreatureDisplayData dispData{};
    if (itDisplayData != displayDataMap_.end()) {
        dispData = itDisplayData->second;
        haveDisplayData = true;
    } else {
        // Some taxi mount display IDs are sparse; recover skins by matching model path.
        std::string lowerMountPath = m2Path;
        std::transform(lowerMountPath.begin(), lowerMountPath.end(), lowerMountPath.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        int bestScore = -1;
        for (const auto& [dispId, data] : displayDataMap_) {
            auto pit = modelIdToPath_.find(data.modelId);
            if (pit == modelIdToPath_.end()) continue;
            std::string p = pit->second;
            std::transform(p.begin(), p.end(), p.begin(),
                           [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            if (p != lowerMountPath) continue;
            int score = 0;
            if (!data.skin1.empty()) {
                std::string p1 = modelDir + data.skin1 + ".blp";
                score += assetManager_->fileExists(p1) ? 30 : 3;
            }
            if (!data.skin2.empty()) {
                std::string p2 = modelDir + data.skin2 + ".blp";
                score += assetManager_->fileExists(p2) ? 20 : 2;
            }
            if (!data.skin3.empty()) {
                std::string p3 = modelDir + data.skin3 + ".blp";
                score += assetManager_->fileExists(p3) ? 10 : 1;
            }
            if (score > bestScore) {
                bestScore = score;
                dispData = data;
                haveDisplayData = true;
            }
        }
        if (haveDisplayData) {
            LOG_INFO("Recovered mount display data by model path for displayId=", mountDisplayId,
                     " skin1='", dispData.skin1, "' skin2='", dispData.skin2,
                     "' skin3='", dispData.skin3, "'");
        }
    }
    if (haveDisplayData) {
        // If this displayId has no skins, try to find another displayId for the same model with skins.
        if (dispData.skin1.empty() && dispData.skin2.empty() && dispData.skin3.empty()) {
            uint32_t sourceModelId = dispData.modelId;
            int bestScore = -1;
            for (const auto& [dispId, data] : displayDataMap_) {
                if (data.modelId != sourceModelId) continue;
                int score = 0;
                if (!data.skin1.empty()) {
                    std::string p = modelDir + data.skin1 + ".blp";
                    score += assetManager_->fileExists(p) ? 30 : 3;
                }
                if (!data.skin2.empty()) {
                    std::string p = modelDir + data.skin2 + ".blp";
                    score += assetManager_->fileExists(p) ? 20 : 2;
                }
                if (!data.skin3.empty()) {
                    std::string p = modelDir + data.skin3 + ".blp";
                    score += assetManager_->fileExists(p) ? 10 : 1;
                }
                if (score > bestScore) {
                    bestScore = score;
                    dispData = data;
                }
            }
            LOG_INFO("Mount skin fallback for displayId=", mountDisplayId,
                     " modelId=", sourceModelId, " skin1='", dispData.skin1,
                     "' skin2='", dispData.skin2, "' skin3='", dispData.skin3, "'");
        }
        const auto* md = charRenderer->getModelData(modelId);
        if (md) {
            LOG_INFO("Mount model textures: ", md->textures.size(), " slots, skin1='", dispData.skin1,
                     "' skin2='", dispData.skin2, "' skin3='", dispData.skin3, "'");
            for (size_t ti = 0; ti < md->textures.size(); ti++) {
                LOG_INFO("  tex[", ti, "] type=", md->textures[ti].type,
                         " filename='", md->textures[ti].filename, "'");
            }

            int replaced = 0;
            for (size_t ti = 0; ti < md->textures.size(); ti++) {
                const auto& tex = md->textures[ti];
                std::string texPath;
                if (tex.type == 11 && !dispData.skin1.empty()) {
                    texPath = modelDir + dispData.skin1 + ".blp";
                } else if (tex.type == 12 && !dispData.skin2.empty()) {
                    texPath = modelDir + dispData.skin2 + ".blp";
                } else if (tex.type == 13 && !dispData.skin3.empty()) {
                    texPath = modelDir + dispData.skin3 + ".blp";
                }
                if (!texPath.empty()) {
                    rendering::VkTexture* skinTex = charRenderer->loadTexture(texPath);
                    if (skinTex) {
                        charRenderer->setModelTexture(modelId, static_cast<uint32_t>(ti), skinTex);
                        LOG_INFO("  Applied skin texture slot ", ti, ": ", texPath);
                        replaced++;
                    } else {
                        LOG_WARNING("  Failed to load skin texture slot ", ti, ": ", texPath);
                    }
                }
            }

            // Force skin textures onto type-0 (hardcoded) slots that have no filename
            if (replaced == 0) {
                for (size_t ti = 0; ti < md->textures.size(); ti++) {
                    const auto& tex = md->textures[ti];
                    if (tex.type == 0 && tex.filename.empty()) {
                        // Empty hardcoded slot — try skin1 then skin2
                        std::string texPath;
                        if (!dispData.skin1.empty() && replaced == 0) {
                            texPath = modelDir + dispData.skin1 + ".blp";
                        } else if (!dispData.skin2.empty()) {
                            texPath = modelDir + dispData.skin2 + ".blp";
                        }
                        if (!texPath.empty()) {
                            rendering::VkTexture* skinTex = charRenderer->loadTexture(texPath);
                            if (skinTex) {
                                charRenderer->setModelTexture(modelId, static_cast<uint32_t>(ti), skinTex);
                                LOG_INFO("  Forced skin on empty hardcoded slot ", ti, ": ", texPath);
                                replaced++;
                            }
                        }
                    }
                }
            }

            // If still no textures, try hardcoded model texture filenames
            if (replaced == 0) {
                for (size_t ti = 0; ti < md->textures.size(); ti++) {
                    if (!md->textures[ti].filename.empty()) {
                        rendering::VkTexture* texId = charRenderer->loadTexture(md->textures[ti].filename);
                        if (texId) {
                            charRenderer->setModelTexture(modelId, static_cast<uint32_t>(ti), texId);
                            LOG_INFO("  Used model embedded texture slot ", ti, ": ", md->textures[ti].filename);
                            replaced++;
                        }
                    }
                }
            }

            // Final fallback for gryphon/wyvern: try well-known skin texture names
            if (replaced == 0 && !md->textures.empty()) {
                std::string lowerMountPath = m2Path;
                std::transform(lowerMountPath.begin(), lowerMountPath.end(), lowerMountPath.begin(),
                               [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
                if (lowerMountPath.find("gryphon") != std::string::npos) {
                    const char* gryphonSkins[] = {
                        "Creature\\Gryphon\\Gryphon_Skin.blp",
                        "Creature\\Gryphon\\Gryphon_Skin01.blp",
                        "Creature\\Gryphon\\GRYPHON_SKIN01.BLP",
                        nullptr
                    };
                    for (const char** p = gryphonSkins; *p; ++p) {
                        rendering::VkTexture* texId = charRenderer->loadTexture(*p);
                        if (texId) {
                            charRenderer->setModelTexture(modelId, 0, texId);
                            LOG_INFO("  Forced gryphon skin fallback: ", *p);
                            replaced++;
                            break;
                        }
                    }
                } else if (lowerMountPath.find("wyvern") != std::string::npos) {
                    const char* wyvernSkins[] = {
                        "Creature\\Wyvern\\Wyvern_Skin.blp",
                        "Creature\\Wyvern\\Wyvern_Skin01.blp",
                        nullptr
                    };
                    for (const char** p = wyvernSkins; *p; ++p) {
                        rendering::VkTexture* texId = charRenderer->loadTexture(*p);
                        if (texId) {
                            charRenderer->setModelTexture(modelId, 0, texId);
                            LOG_INFO("  Forced wyvern skin fallback: ", *p);
                            replaced++;
                            break;
                        }
                    }
                }
            }
            LOG_INFO("Mount texture setup: ", replaced, " textures applied");
        }
    }

    mountModelId_ = modelId;

    // Create mount instance at player position
    glm::vec3 mountPos = renderer_->getCharacterPosition();
    float yawRad = glm::radians(renderer_->getCharacterYaw());
    uint32_t instanceId = charRenderer->createInstance(modelId, mountPos,
        glm::vec3(0.0f, 0.0f, yawRad), 1.0f);

    if (instanceId == 0) {
        LOG_WARNING("Failed to create mount instance");
        return;
    }

    mountInstanceId_ = instanceId;

    // Compute height offset — place player above mount's back
    // Use tight bounds from actual vertices (M2 header bounds can be inaccurate)
    const auto* modelData = charRenderer->getModelData(modelId);
    float heightOffset = 1.8f;
    if (modelData && !modelData->vertices.empty()) {
        float minZ =  std::numeric_limits<float>::max();
        float maxZ = -std::numeric_limits<float>::max();
        for (const auto& v : modelData->vertices) {
            if (v.position.z < minZ) minZ = v.position.z;
            if (v.position.z > maxZ) maxZ = v.position.z;
        }
        float extentZ = maxZ - minZ;
        LOG_INFO("Mount tight bounds: minZ=", minZ, " maxZ=", maxZ, " extentZ=", extentZ);
        if (extentZ > 0.5f) {
            // Saddle point is roughly 75% up the model, measured from model origin
            heightOffset = maxZ * 0.8f;
            if (heightOffset < 1.0f) heightOffset = extentZ * 0.75f;
            if (heightOffset < 1.0f) heightOffset = 1.8f;
        }
    }

    if (auto* ac = renderer_->getAnimationController()) ac->setMounted(instanceId, mountDisplayId, heightOffset, m2Path);

    // For taxi mounts, start with flying animation; for ground mounts, start with stand
    bool isTaxi = gameHandler_ && gameHandler_->isOnTaxiFlight();
    uint32_t startAnim = rendering::anim::STAND;
    if (isTaxi) {
        // Try WotLK fly anims first, then Vanilla-friendly fallbacks
        using namespace rendering::anim;
        uint32_t taxiCandidates[] = {FLY_FORWARD, FLY_IDLE, FLY_RUN_2, FLY_SPELL, FLY_RISE, SPELL_KNEEL_LOOP, FLY_CUSTOM_SPELL_10, DEAD, RUN};
        for (uint32_t anim : taxiCandidates) {
            if (charRenderer->hasAnimation(instanceId, anim)) {
                startAnim = anim;
                break;
            }
        }
        // If none found, startAnim stays 0 (Stand/hover) which is fine for flying creatures
    }
    charRenderer->playAnimation(instanceId, startAnim, true);

    LOG_INFO("processPendingMount: DONE displayId=", mountDisplayId, " model=", m2Path, " heightOffset=", heightOffset);
}

bool EntitySpawner::loadRemoteMountModel(uint32_t displayId, uint32_t& modelId,
                                         std::string& modelPath, float& riderHeight) {
    modelId = 0;
    riderHeight = 1.8f;
    if (!renderer_ || !renderer_->getCharacterRenderer() || !assetManager_) return false;
    auto* cr = renderer_->getCharacterRenderer();

    modelPath = getModelPathForDisplayId(displayId);
    if (modelPath.empty()) {
        LOG_WARNING("Remote player mount has no model path: displayId=", displayId);
        return false;
    }

    auto cached = displayIdModelCache_.find(displayId);
    if (cached != displayIdModelCache_.end() && cr->getModelData(cached->second)) {
        modelId = cached->second;
    } else {
        auto m2Data = assetManager_->readFile(modelPath);
        if (m2Data.empty()) return false;
        pipeline::M2Model model = pipeline::M2Loader::load(m2Data);
        if (model.name.empty()) model.name = modelPath;
        if (model.vertices.empty()) return false;

        if (model.version >= 264 && modelPath.size() >= 3) {
            std::string skinPath = modelPath.substr(0, modelPath.size() - 3) + "00.skin";
            auto skinData = assetManager_->readFile(skinPath);
            if (!skinData.empty()) pipeline::M2Loader::loadSkin(skinData, model);
        }
        if (!model.isValid()) return false;

        const std::string basePath = modelPath.substr(0, modelPath.size() - 3);
        for (uint32_t si = 0; si < model.sequences.size(); ++si) {
            if (model.sequences[si].flags & 0x20) continue;
            const uint32_t animId = model.sequences[si].id;
            if (animId != rendering::anim::STAND && animId != rendering::anim::WALK &&
                animId != rendering::anim::RUN && animId != rendering::anim::FLY_IDLE &&
                animId != rendering::anim::FLY_FORWARD) continue;
            char animFile[256];
            snprintf(animFile, sizeof(animFile), "%s%04u-%02u.anim", basePath.c_str(),
                     animId, model.sequences[si].variationIndex);
            auto animData = assetManager_->readFileOptional(animFile);
            if (!animData.empty()) pipeline::M2Loader::loadAnimFile(m2Data, animData, si, model);
        }

        modelId = nextCreatureModelId_++;
        if (!cr->loadModel(model, modelId)) return false;
        displayIdModelCache_[displayId] = modelId;

        // CreatureDisplayInfo supplies the replaceable mount skins. Mount model
        // IDs are cached per display ID, so applying them to the model is safe.
        auto displayIt = displayDataMap_.find(displayId);
        if (displayIt != displayDataMap_.end()) {
            CreatureDisplayData skin = displayIt->second;
            if (skin.skin1.empty() && skin.skin2.empty() && skin.skin3.empty()) {
                for (const auto& [candidateId, candidate] : displayDataMap_) {
                    (void)candidateId;
                    if (candidate.modelId == skin.modelId &&
                        (!candidate.skin1.empty() || !candidate.skin2.empty() || !candidate.skin3.empty())) {
                        skin = candidate;
                        break;
                    }
                }
            }
            const size_t slash = modelPath.find_last_of("\\/");
            const std::string dir = slash == std::string::npos ? "" : modelPath.substr(0, slash + 1);
            if (const auto* md = cr->getModelData(modelId)) {
                for (size_t ti = 0; ti < md->textures.size(); ++ti) {
                    std::string name;
                    if (md->textures[ti].type == 11) name = skin.skin1;
                    else if (md->textures[ti].type == 12) name = skin.skin2;
                    else if (md->textures[ti].type == 13) name = skin.skin3;
                    if (name.empty()) continue;
                    if (auto* texture = cr->loadTexture(dir + name + ".blp")) {
                        cr->setModelTexture(modelId, static_cast<uint32_t>(ti), texture);
                    }
                }
            }
        }
    }

    if (const auto* md = cr->getModelData(modelId); md && !md->vertices.empty()) {
        float minZ = std::numeric_limits<float>::max();
        float maxZ = -std::numeric_limits<float>::max();
        for (const auto& vertex : md->vertices) {
            minZ = std::min(minZ, vertex.position.z);
            maxZ = std::max(maxZ, vertex.position.z);
        }
        const float extent = maxZ - minZ;
        if (extent > 0.5f) {
            riderHeight = maxZ * 0.8f;
            if (riderHeight < 1.0f) riderHeight = extent * 0.75f;
            if (riderHeight < 1.0f) riderHeight = 1.8f;
        }
    }
    return modelId != 0;
}

void EntitySpawner::processPendingRemotePlayerMounts() {
    if (pendingRemotePlayerMounts_.empty() || !renderer_) return;
    auto* cr = renderer_->getCharacterRenderer();
    if (!cr) return;

    // Mount model loading can touch disk and upload GPU resources. Process at
    // most one transition per frame, consistent with the other spawn queues.
    for (auto it = pendingRemotePlayerMounts_.begin();
         it != pendingRemotePlayerMounts_.end(); ++it) {
        const uint64_t guid = it->first;
        const uint32_t displayId = it->second;

        if (displayId == 0) {
            removeRemotePlayerMount(guid);
            pendingRemotePlayerMounts_.erase(it);
            return;
        }
        auto playerIt = playerInstances_.find(guid);
        if (playerIt == playerInstances_.end()) continue; // initial fields can precede rendering

        auto current = remotePlayerMounts_.find(guid);
        if (current != remotePlayerMounts_.end() && current->second.displayId == displayId) {
            pendingRemotePlayerMounts_.erase(it);
            return;
        }
        removeRemotePlayerMount(guid);

        uint32_t modelId = 0;
        float riderHeight = 0.0f;
        std::string modelPath;
        if (!loadRemoteMountModel(displayId, modelId, modelPath, riderHeight)) {
            LOG_WARNING("Failed to load remote player mount: guid=0x", std::hex, guid,
                        std::dec, " displayId=", displayId);
            pendingRemotePlayerMounts_.erase(it);
            return;
        }

        glm::vec3 pos(0.0f);
        cr->getInstancePosition(playerIt->second, pos);
        uint32_t mountInstance = cr->createInstance(modelId, pos, glm::vec3(0.0f), 1.0f);
        if (mountInstance != 0) {
            const bool moving = gameHandler_ && [&] {
                auto entity = gameHandler_->getEntityManager().getEntity(guid);
                return entity && entity->isActivelyMoving();
            }();
            const bool flying = creatureFlyingState_.count(guid) > 0;
            const bool walking = creatureWalkingState_.count(guid) > 0;
            uint32_t mountAnim = moving
                ? (flying ? rendering::anim::FLY_FORWARD
                          : (walking ? rendering::anim::WALK : rendering::anim::RUN))
                : (flying ? rendering::anim::FLY_IDLE : rendering::anim::STAND);
            if (!cr->hasAnimation(mountInstance, mountAnim)) {
                mountAnim = moving ? rendering::anim::RUN : rendering::anim::STAND;
            }
            cr->playAnimation(mountInstance, mountAnim, true);
            cr->playAnimation(playerIt->second, rendering::anim::MOUNT, true);
            remotePlayerMounts_[guid] = {displayId, modelId, mountInstance, riderHeight};
            LOG_INFO("Remote player mounted: guid=0x", std::hex, guid, std::dec,
                     " displayId=", displayId, " riderHeight=", riderHeight,
                     " model=", modelPath);
        }
        pendingRemotePlayerMounts_.erase(it);
        return;
    }
}

void EntitySpawner::erasePendingGuidIfUnqueued(uint64_t guid) {
    for (const auto& pending : pendingCreatureSpawns_) {
        if (pending.guid == guid) return;
    }
    pendingCreatureSpawnGuids_.erase(guid);
}

void EntitySpawner::despawnCreature(uint64_t guid) {
    // If this guid is a PLAYER, it will be tracked in playerInstances_.
    // Route to the correct despawn path so we don't leak instances.
    if (playerInstances_.count(guid)) {
        despawnPlayer(guid);
        return;
    }

    pendingCreatureSpawnGuids_.erase(guid);
    requestedCreatureDisplayIds_.erase(guid);
    creatureActiveEmotes_.erase(guid);
    creatureSpawnRetryDeadlines_.erase(guid);
    // Cleared with the rest, so the map does not grow and a creature that
    // comes back into range starts with its full allowance again.
    creatureSpawnRetryWindowsUsed_.erase(guid);
    creaturePermanentFailureGuids_.erase(guid);
    deadCreatureGuids_.erase(guid);

    auto it = creatureInstances_.find(guid);
    if (it == creatureInstances_.end()) return;

    if (renderer_ && renderer_->getCharacterRenderer()) {
        renderer_->getCharacterRenderer()->removeInstance(it->second);
    }

    creatureInstances_.erase(it);
    creatureModelIds_.erase(guid);
    creatureDisplayIds_.erase(guid);
    creatureRenderPosCache_.erase(guid);
    creatureWeaponsAttached_.erase(guid);
    creatureWeaponAttachAttempts_.erase(guid);
    creatureWasMoving_.erase(guid);
    creatureWasSwimming_.erase(guid);
    creatureWasFlying_.erase(guid);
    creatureWasWalking_.erase(guid);
    creatureSwimmingState_.erase(guid);
    creatureWalkingState_.erase(guid);
    creatureFlyingState_.erase(guid);
    creatureWasStealthed_.erase(guid);

    LOG_DEBUG("Despawned creature: guid=0x", std::hex, guid, std::dec);
}

namespace {

// Game object types whose pose is server state rather than a looping idle: they
// hold one frame until the server says otherwise (a door stands open or shut, a
// chest sits closed until it is opened), so playing their sequence on a loop
// would animate them open over and over. Every other type plays its idle
// continuously, which is what retail does — fishing pools circle their fish,
// braziers gutter, banners wave.
bool gameObjectPoseIsStateDriven(uint32_t goType) {
    switch (goType) {
        case 0:   // DOOR
        case 1:   // BUTTON
        case 3:   // CHEST
        case 6:   // TRAP
        case 10:  // GOOBER
        case 33:  // DESTRUCTIBLE_BUILDING
        case 35:  // TRAPDOOR
            return true;
        default:
            return false;
    }
}

} // namespace

void EntitySpawner::applyGameObjectAnimationPolicy(uint64_t guid, uint32_t entry,
                                                   uint32_t instanceId) {
    auto* m2Renderer = renderer_ ? renderer_->getM2Renderer() : nullptr;
    if (!m2Renderer) return;

    const game::GameObjectQueryResponseData* info =
        (gameHandler_ && entry != 0) ? gameHandler_->getCachedGameObjectInfo(entry) : nullptr;
    if (!info) {
        // The type has not arrived yet. Freeze for now — a door caught mid-swing
        // is worse than a pool of still fish — and revisit in
        // onGameObjectInfoReceived once the query response lands.
        m2Renderer->setInstanceAnimationFrozen(instanceId, true);
        if (entry != 0) gameObjectPendingAnimPolicy_[entry].push_back(instanceId);
        return;
    }

    const bool freeze = gameObjectPoseIsStateDriven(info->type);
    m2Renderer->setInstanceAnimationFrozen(instanceId, freeze);
    LOG_DEBUG("GO animation policy: guid=0x", std::hex, guid, std::dec,
              " entry=", entry, " type=", info->type, " frozen=", freeze);
}

void EntitySpawner::onGameObjectInfoReceived(uint32_t entry) {
    auto it = gameObjectPendingAnimPolicy_.find(entry);
    if (it == gameObjectPendingAnimPolicy_.end()) return;
    auto* m2Renderer = renderer_ ? renderer_->getM2Renderer() : nullptr;
    const game::GameObjectQueryResponseData* info =
        (gameHandler_ && m2Renderer) ? gameHandler_->getCachedGameObjectInfo(entry) : nullptr;
    if (info && !gameObjectPoseIsStateDriven(info->type)) {
        for (uint32_t instanceId : it->second) {
            // No-op for instances that despawned while the query was in flight.
            m2Renderer->setInstanceAnimationFrozen(instanceId, false);
        }
        LOG_DEBUG("GO animation policy resolved: entry=", entry,
                  " type=", info->type, " unfroze ", it->second.size(), " instance(s)");
    }
    gameObjectPendingAnimPolicy_.erase(it);
}

void EntitySpawner::despawnGameObject(uint64_t guid) {
    pendingTransportDoodadBatches_.erase(
        std::remove_if(pendingTransportDoodadBatches_.begin(), pendingTransportDoodadBatches_.end(),
                       [guid](const PendingTransportDoodadBatch& b) { return b.guid == guid; }),
        pendingTransportDoodadBatches_.end());

    auto it = gameObjectInstances_.find(guid);
    if (it == gameObjectInstances_.end()) return;

    if (gameHandler_ && gameHandler_->isTransportGuid(guid)) {
        if (auto* transportManager = gameHandler_->getTransportManager()) {
            if (auto* transport = transportManager->getTransport(guid)) {
                const bool isDeeprunTram =
                    game::TransportManager::isDeeprunTramTransport(*transport);
                if (transport->isM2 && isDeeprunTram && game::isPreWotlk()) {
                    LOG_DEBUG("Keeping Deeprun tram render instance through server despawn: guid=0x",
                                std::hex, guid, std::dec,
                                " entry=", transport->entry,
                                " displayId=", transport->displayId,
                                " pathId=", transport->pathId,
                                " instanceId=", it->second.instanceId);
                    return;
                }
            }
        }
    }

    if (renderer_) {
        if (it->second.isWmo) {
            if (auto* wmoRenderer = renderer_->getWMORenderer()) {
                wmoRenderer->removeInstance(it->second.instanceId);
            }
        } else {
            if (auto* m2Renderer = renderer_->getM2Renderer()) {
                m2Renderer->removeInstance(it->second.instanceId);
            }
        }
    }

    gameObjectInstances_.erase(it);

    LOG_DEBUG("Despawned gameobject: guid=0x", std::hex, guid, std::dec);
}

bool EntitySpawner::loadWeaponM2(const std::string& m2Path, pipeline::M2Model& outModel) {
    auto m2Data = assetManager_->readFile(m2Path);
    if (m2Data.empty()) return false;
    outModel = pipeline::M2Loader::load(m2Data);
    // Load skin (WotLK+ M2 format): strip .m2, append 00.skin
    std::string skinPath = m2Path;
    size_t dotPos = skinPath.rfind('.');
    if (dotPos != std::string::npos) skinPath = skinPath.substr(0, dotPos);
    skinPath += "00.skin";
    auto skinData = assetManager_->readFile(skinPath);
    if (!skinData.empty() && outModel.version >= 264)
        pipeline::M2Loader::loadSkin(skinData, outModel);
    return outModel.isValid();
}


} // namespace core
} // namespace wowee
