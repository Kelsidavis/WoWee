#include "auth/auth_packets.hpp"
#include "auth/crypto.hpp"
#include "auth/integrity.hpp"
#include "auth/srp.hpp"
#include "network/tcp_socket.hpp"
#include "network/packet.hpp"
#include "core/logger.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using namespace wowee;

static void usage() {
    std::cerr
        << "Usage:\n"
        << "  auth_login_probe <host> <port> <account> <major> <minor> <patch> <build> <proto> <locale> \\\n"
        << "                 (--password <pass> | --hash <hexsha1>) [--proof legacy|v8|auto]\n"
        << "\n"
        << "Notes:\n"
        << "  - --hash expects SHA1(UPPER(user):UPPER(pass)) in hex.\n"
        << "  - This tool only probes auth; it does not connect to world.\n";
}

static std::vector<uint8_t> hexToBytes(const std::string& hex) {
    std::vector<uint8_t> out;
    std::string h;
    h.reserve(hex.size());
    for (char c : hex) {
        if (!std::isspace(static_cast<unsigned char>(c))) h.push_back(c);
    }
    if (h.size() % 2 != 0) throw std::runtime_error("hex length must be even");
    out.reserve(h.size() / 2);
    for (size_t i = 0; i < h.size(); i += 2) {
        auto byteStr = h.substr(i, 2);
        uint8_t b = static_cast<uint8_t>(std::stoul(byteStr, nullptr, 16));
        out.push_back(b);
    }
    return out;
}

static std::string upperAscii(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    return s;
}

enum class ProofFormat { Auto, Legacy, V8 };
enum class CrcAFormat { Wire, BigEndian };
enum class WireAFormat { Little, Big };

int main(int argc, char** argv) {
    if (argc < 11) {
        usage();
        return 2;
    }

    const std::string host = argv[1];
    const int port = std::atoi(argv[2]);
    const std::string account = argv[3];
    const int major = std::atoi(argv[4]);
    const int minor = std::atoi(argv[5]);
    const int patch = std::atoi(argv[6]);
    const int build = std::atoi(argv[7]);
    const int proto = std::atoi(argv[8]);
    const std::string locale = argv[9];

    std::string password;
    std::vector<uint8_t> authHash;
    bool havePassword = false;
    bool haveHash = false;
    ProofFormat proofFmt = ProofFormat::Auto;
    CrcAFormat crcA = CrcAFormat::Wire;
    WireAFormat wireA = WireAFormat::Little;
    std::string integrityExe = "WoW.exe";
    bool serverValuesBigEndian = false;
    std::string miscDir = "Data/misc";
    bool useHashedK = false;
    bool hashBigEndian = false;

    for (int i = 10; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--password" && i + 1 < argc) {
            password = argv[++i];
            havePassword = true;
            continue;
        }
        if (a == "--hash" && i + 1 < argc) {
            authHash = hexToBytes(argv[++i]);
            haveHash = true;
            continue;
        }
        if (a == "--proof" && i + 1 < argc) {
            std::string v = argv[++i];
            if (v == "auto") proofFmt = ProofFormat::Auto;
            else if (v == "legacy") proofFmt = ProofFormat::Legacy;
            else if (v == "v8") proofFmt = ProofFormat::V8;
            else {
                std::cerr << "Unknown --proof value: " << v << "\n";
                return 2;
            }
            continue;
        }
        if (a == "--crc-a" && i + 1 < argc) {
            std::string v = argv[++i];
            if (v == "wire") crcA = CrcAFormat::Wire;
            else if (v == "be") crcA = CrcAFormat::BigEndian;
            else {
                std::cerr << "Unknown --crc-a value: " << v << " (expected wire|be)\n";
                return 2;
            }
            continue;
        }
        if (a == "--integrity-exe" && i + 1 < argc) {
            integrityExe = argv[++i];
            continue;
        }
        if (a == "--misc-dir" && i + 1 < argc) {
            miscDir = argv[++i];
            continue;
        }
        if (a == "--server-values" && i + 1 < argc) {
            std::string v = argv[++i];
            if (v == "le") serverValuesBigEndian = false;
            else if (v == "be") serverValuesBigEndian = true;
            else {
                std::cerr << "Unknown --server-values value: " << v << " (expected le|be)\n";
                return 2;
            }
            continue;
        }
        if (a == "--wire-a" && i + 1 < argc) {
            std::string v = argv[++i];
            if (v == "le") wireA = WireAFormat::Little;
            else if (v == "be") wireA = WireAFormat::Big;
            else {
                std::cerr << "Unknown --wire-a value: " << v << " (expected le|be)\n";
                return 2;
            }
            continue;
        }
        if (a == "--k" && i + 1 < argc) {
            std::string v = argv[++i];
            if (v == "3") useHashedK = false;
            else if (v == "hashed") useHashedK = true;
            else {
                std::cerr << "Unknown --k value: " << v << " (expected 3|hashed)\n";
                return 2;
            }
            continue;
        }
        if (a == "--hash-endian" && i + 1 < argc) {
            std::string v = argv[++i];
            if (v == "le") hashBigEndian = false;
            else if (v == "be") hashBigEndian = true;
            else {
                std::cerr << "Unknown --hash-endian value: " << v << " (expected le|be)\n";
                return 2;
            }
            continue;
        }
        std::cerr << "Unknown arg: " << a << "\n";
        return 2;
    }

    if (!havePassword && !haveHash) {
        std::cerr << "Must supply --password or --hash\n";
        return 2;
    }

    auth::ClientInfo info;
    info.majorVersion = static_cast<uint8_t>(major);
    info.minorVersion = static_cast<uint8_t>(minor);
    info.patchVersion = static_cast<uint8_t>(patch);
    info.build = static_cast<uint16_t>(build);
    info.protocolVersion = static_cast<uint8_t>(proto);
    info.locale = locale;
    info.platform = "x86";
    info.os = "Win";

    std::atomic<bool> done{false};
    std::atomic<bool> sawDisconnect{false};
    std::atomic<bool> challengeOk{false};
    std::atomic<int> proofStatus{-1};
    std::atomic<int> chalCode{-1};

    network::TCPSocket sock;
    std::unique_ptr<auth::SRP> srp;
    uint8_t securityFlags = 0;
    uint32_t pinSeed = 0;
    std::array<uint8_t, 16> pinSalt{};
    std::array<uint8_t, 16> checksumSalt{};

    auto sendProof = [&]() {
        if (!srp) return;
        auto A = srp->getA();
        if (wireA == WireAFormat::Big) {
            std::reverse(A.begin(), A.end());
        }
        auto M1 = srp->getM1();

        ProofFormat fmt = proofFmt;
        if (fmt == ProofFormat::Auto) {
            fmt = (info.protocolVersion < 8) ? ProofFormat::Legacy : ProofFormat::V8;
        }

        // Try to compute the classic client integrity hash using local Data/misc.
        std::array<uint8_t, 20> crcHash{};
        const std::array<uint8_t, 20>* crcHashPtr = nullptr;
        {
            std::string err;
            std::vector<uint8_t> crcABytes = A;
            if (crcA == CrcAFormat::BigEndian) {
                std::reverse(crcABytes.begin(), crcABytes.end());
            }
            if (auth::computeIntegrityHashWin32WithExe(checksumSalt, crcABytes, miscDir, integrityExe, crcHash, err)) {
                crcHashPtr = &crcHash;
                std::cerr << "Computed integrity hash using " << miscDir << " (" << integrityExe << ")\n";
            } else {
                std::cerr << "Integrity hash not computed: " << err << "\n";
            }
        }

        if (fmt == ProofFormat::Legacy) {
            auto pkt = auth::LogonProofPacket::buildLegacy(A, M1, crcHashPtr);
            sock.send(pkt);
            std::cerr << "Sent LOGON_PROOF legacy (proto=" << (int)info.protocolVersion << ")\n";
        } else {
            auto pkt = auth::LogonProofPacket::build(A, M1, securityFlags, crcHashPtr, nullptr, nullptr);
            sock.send(pkt);
            std::cerr << "Sent LOGON_PROOF v8 (secFlags=0x" << std::hex << (int)securityFlags << std::dec << ")\n";
        }
    };

    sock.setPacketCallback([&](const network::Packet& p) {
        network::Packet pkt = p;
        if (pkt.getSize() < 1) return;

        uint8_t opcode = pkt.readUInt8();
        if (opcode == static_cast<uint8_t>(auth::AuthOpcode::LOGON_CHALLENGE)) {
            auth::LogonChallengeResponse resp{};
            if (!auth::LogonChallengeResponseParser::parse(pkt, resp)) {
                std::cerr << "Challenge parse failed\n";
                done = true;
                return;
            }
            chalCode = static_cast<int>(resp.result);
            if (!resp.isSuccess()) {
                std::cerr << "Challenge FAIL: " << auth::getAuthResultString(resp.result)
                          << " (0x" << std::hex << (int)resp.result << std::dec << ")\n";
                done = true;
                return;
            }

            challengeOk = true;
            securityFlags = resp.securityFlags;
            pinSeed = resp.pinGridSeed;
            pinSalt = resp.pinSalt;
            checksumSalt = resp.checksumSalt;

            srp = std::make_unique<auth::SRP>();
            srp->setUseHashedK(useHashedK);
            srp->setHashBigEndian(hashBigEndian);
            if (haveHash) {
                srp->initializeWithHash(account, authHash);
            } else {
                srp->initialize(account, password);
            }
            if (serverValuesBigEndian) {
                auto rev = [](std::vector<uint8_t> v) {
                    std::reverse(v.begin(), v.end());
                    return v;
                };
                srp->feed(rev(resp.B), rev(resp.g), rev(resp.N), rev(resp.salt));
            } else {
                srp->feed(resp.B, resp.g, resp.N, resp.salt);
            }

            sendProof();
            return;
        }

        if (opcode == static_cast<uint8_t>(auth::AuthOpcode::LOGON_PROOF)) {
            auth::LogonProofResponse resp{};
            if (!auth::LogonProofResponseParser::parse(pkt, resp)) {
                std::cerr << "Proof parse failed\n";
                done = true;
                return;
            }
            proofStatus = resp.status;
            if (resp.isSuccess()) {
                std::cerr << "Proof SUCCESS\n";
            } else {
                std::cerr << "Proof FAIL status=0x" << std::hex << (int)resp.status << std::dec << "\n";
            }
            done = true;
            return;
        }
    });

    if (!sock.connect(host, static_cast<uint16_t>(port))) {
        std::cerr << "Connect failed\n";
        return 3;
    }

    auto chal = auth::LogonChallengePacket::build(account, info);
    sock.send(chal);

    auto start = std::chrono::steady_clock::now();
    while (!done) {
        sock.update();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        if (!sock.isConnected() && !done) {
            sawDisconnect = true;
            done = true;
            break;
        }

        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() > 6000) {
            break;
        }
    }

    sock.disconnect();

    if (!done && sock.isConnected()) {
        std::cerr << "Timeout\n";
        return 4;
    }

    if (sawDisconnect && challengeOk && proofStatus.load() < 0) {
        std::cerr << "Server disconnected after challenge (no proof response parsed)\n";
        return 6;
    }

    if (chalCode.load() >= 0 && chalCode.load() != 0) return chalCode.load();
    if (proofStatus.load() >= 0) return proofStatus.load();
    return 0;
}
