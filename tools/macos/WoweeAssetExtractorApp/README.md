# Projet Xcode de l'extracteur d'assets

Ce dossier ne contient **pas de code** : il contient la recette qui engendre un
projet Xcode pour l'application dont les sources vivent dans le paquet SwiftPM
voisin, [`../WoweeAssetExtractor`](../WoweeAssetExtractor).

```bash
xcodegen generate                    # -> WoweeAssetExtractor.xcodeproj
open WoweeAssetExtractor.xcodeproj
```

`xcodegen` s'installe avec `brew install xcodegen`.

## Pourquoi les sources ne sont pas ici

Une copie des sources dans un projet Xcode et une autre dans le paquet SwiftPM
divergent le jour ou l'une des deux est corrigee seule. Le projet reference donc
les fichiers **en place**, par des chemins relatifs :

| Cible Xcode | Sources |
|---|---|
| `ExtractorKit` (bibliotheque statique) | `../WoweeAssetExtractor/Sources/ExtractorKit` |
| `WoweeAssetExtractor` (application) | `../WoweeAssetExtractor/Sources/WoweeAssetExtractor` |
| `ExtractorKitTests` (tests) | `../WoweeAssetExtractor/Tests/ExtractorKitTests` |

Editer un fichier depuis Xcode edite le fichier du paquet. `swift build` et
`swift test` continuent donc de fonctionner a l'identique - ce dont la CI depend,
puisqu'elle n'ouvre pas Xcode.

## Pourquoi le .xcodeproj n'est pas versionne

Il se reengendre en une commande, et un `project.pbxproj` commite entre en
conflit des que deux branches ajoutent un fichier. C'est `project.yml` qui est
versionne, lu, et relu en revue - une trentaine de lignes au lieu de huit cents.

Un fichier ajoute au paquet n'apparait dans Xcode qu'apres un nouveau
`xcodegen generate`.

## Ce que le projet fait, que `swift build` ne fait pas

- **Lancer et deboguer** l'application depuis Xcode (points d'arret, Instruments,
  hierarchie des vues), la ou SwiftPM ne produit qu'un binaire nu.
- **Assembler le bundle** `.app` avec son `Info.plist` et sa signature, sans
  passer par `../WoweeAssetExtractor/make_app.sh`.
- **Executer les 21 tests** au ⌘U.

Les deux chemins restent valables et donnent le meme resultat :

```bash
cd ../WoweeAssetExtractor && swift test    # 21 tests
xcodebuild -project WoweeAssetExtractor.xcodeproj \
           -scheme WoweeAssetExtractor test   # les memes 21
```

## Signature

Le projet signe en **ad-hoc** (`CODE_SIGN_IDENTITY = "-"`), comme `make_app.sh` :
suffisant pour lancer en local, et surtout sans equipe de developpement a
renseigner pour que le build passe. Pour distribuer, renseigner une identite
Developer ID dans les reglages de la cible - ou passer par `make_app.sh
--identity`, qui reste le chemin de la CI.

L'application n'est pas mise en bac a sable : elle lit un dossier arbitraire
choisi par l'utilisateur et ecrit dans `~/Library/Application Support/Wowee/Data`.
