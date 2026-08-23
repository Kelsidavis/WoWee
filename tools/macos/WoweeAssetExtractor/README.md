# Extracteur d'assets WoWee (SwiftUI)

Une application macOS native qui prépare les données du client WoW pour WoWee :
on dépose le dossier `Data`, elle extrait, et elle montre où elle en est.

Elle **coexiste** avec `Wowee Asset Extractor.app` (l'applet AppleScript de
`tools/macos/`), qui reste inchangée et reste ce que la CI publie aujourd'hui.
Cette version-ci est une alternative complète, pas un remplacement en place.

```bash
./make_app.sh                       # build/WoweeAssetExtractor.app
open build/WoweeAssetExtractor.app
```

## Ce qu'elle fait

- **Glisser-déposer** du dossier `Data`, ou sélection par le sélecteur de fichiers.
- **Vérification immédiate** du dossier déposé : le nombre d'archives MPQ trouvées
  s'affiche avant de lancer quoi que ce soit, sous-dossiers de locale compris
  (`enUS/`, `frFR/`…). Un dossier sans MPQ est refusé tout de suite, plutôt qu'au
  bout d'une extraction engagée.
- **Barre de progression déterminée** en quatre phases, alimentée par la sortie
  réelle de `asset_extract` — pas un défilement décoratif.
- **Estimation du temps restant**, calculée sur le débit observé, affichée
  seulement pendant l'extraction où elle a un sens.
- **Choix de l'extension** (auto-détection par défaut) et de la destination.
- **Annulation** à tout moment, **journal** dépliable, et révélation du résultat
  dans le Finder à la fin.
- **Avertissement d'espace disque** avant de lancer : une extraction complète
  pèse environ 18 Go.

## Conformité aux Human Interface Guidelines

- **Barre de menus complète.** Toute action accessible à la souris a son
  équivalent clavier et son entrée de menu : ouvrir un dossier (⌘O), extraire
  (⌘R), annuler (⌘.), afficher le journal (⌘L), révéler dans le Finder (⇧⌘R).
  Les entrées s'activent et se désactivent selon l'état, et le libellé du
  journal dit ce qu'il va faire plutôt que ce dont il parle.
- **Fenêtre librement redimensionnable**, avec un plancher qui garde la mise en
  page utilisable. Position et taille sont restaurées d'un lancement à l'autre.
- **Échap annule** l'extraction ; **Entrée** déclenche l'action par défaut.
- **Menus contextuels** sur la zone de dépôt (choisir, révéler, retirer) et sur
  le journal (copier).
- **Survol** distinct de l'état de dépôt sur la zone cible.
- **VoiceOver** : la zone de dépôt est annoncée comme un seul contrôle, avec sa
  valeur et une action ; la progression est lue avec son pourcentage, que la
  barre seule ne dit pas à qui ne la voit pas.
- **Réduire les animations**, **Augmenter le contraste** et **Texte en gras**
  sont respectés — sous contraste augmenté, la bordure de la zone de dépôt
  passe en couleur pleine, faute de quoi le seul repère de la cible disparaît.
- **Couleurs sémantiques** uniquement, donc mode sombre correct par
  construction.

## Comment la progression est calculée

`asset_extract` annonce ses phases sur stdout. Le parseur
(`Sources/ExtractorKit/ExtractionProgress.swift`) les lit :

| Sortie de l'extracteur | Source | Phase |
|---|---|---|
| `Found N MPQ archives` | `extractor.cpp:602` | début du scan, total connu |
| `  Scanning: <archive>` | `extractor.cpp:628` | scan, une par archive |
| `Enumerated N unique files` | `extractor.cpp:659` | inventaire |
| `\r  Extracted X / Y files...` | `extractor.cpp:851` | extraction, X/Y |
| `Wrote manifest: …` | `extractor.cpp:923` | finalisation |

Deux détails rendent ce parsing moins trivial qu'il n'en a l'air, et les deux
sont couverts par des tests :

- La ligne de progression est réécrite **avec un retour chariot et sans saut de
  ligne**. Découper le flux sur `\n` seul ne la voit jamais ; et attendre un
  terminateur affiche une barre avec un tick de retard, le dernier n'arrivant
  jamais. Le parseur lit donc aussi la ligne partielle en attente, sans la
  consommer — et seulement pour le motif qu'on peut appliquer deux fois sans
  fausser un compteur.
- Un tube n'est pas un terminal : une ligne peut arriver coupée en deux. Le
  parseur conserve la fin incomplète entre deux lectures.

## Différences avec l'applet livrée

Trois écarts sont volontaires, chacun corrigeant un comportement relevé dans
`tools/macos/asset_extractor_launcher.sh` :

1. **Recherche de `Wowee.app`** — l'applet accepte n'importe quel dossier nommé
   `Wowee.app` (`[ -d ]`). Une installation ancienne ou incomplète dans
   `/Applications` masque alors une bonne installation dans `~/Applications`.
   Ici la recherche exige un `Contents/MacOS/asset_extract` exécutable.
2. **Semis des profils d'expansion** — l'applet fait un `ditto`, qui écrase.
   Or `expansion.json` est un fichier que la documentation invite à éditer à la
   main (`CONTRIBUTING.md`, `wardenRsaModulus` d'un serveur privé). Ici seuls
   les fichiers absents sont copiés ; ce qui existe est laissé intact.
3. **Visibilité des erreurs** — l'applet est un agent (`LSUIElement`), donc un
   échec avant l'ouverture du Terminal ne produit rien à l'écran. Celle-ci est
   une application normale, avec un état d'erreur et un journal.

## Développement

```bash
swift build     # compile
swift test      # 21 tests, sans fenêtre
```

Le code est séparé en deux cibles à dessein : `ExtractorKit` contient tout ce
qui se teste sans interface (localisation du binaire, parsing, inspection d'un
dossier, semis des profils), et `WoweeAssetExtractor` n'est que la coquille
SwiftUI autour.

## Signature

`make_app.sh` signe en ad-hoc par défaut, ce qui suffit en local. Pour une
distribution :

```bash
./make_app.sh --identity "Developer ID Application: … (TEAMID)" --icon path/to/AppIcon.icns
```

L'application n'est pas sandboxée : elle lit un dossier arbitraire choisi par
l'utilisateur et écrit dans `~/Library/Application Support/Wowee/Data`, le
chemin que `src/main.cpp` lit lorsque `WOW_DATA_PATH` n'est pas défini.
