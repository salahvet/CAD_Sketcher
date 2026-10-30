# Pont Al-Tasrif → CAD Sketcher (API v1.0)

Al-Tasrif stocke (références, données de modélisation, gabarits). Sketcher exécute.
Le seul point d'entrée est le module `api.py` de Sketcher :

```python
import importlib
api = importlib.import_module("bl_ext.user_default.CAD_Sketcher.api")
assert api.API_VERSION[0] == 1
```

Toutes les longueurs sont en **mm**, les angles en **degrés**. La conversion vers les unités Blender (échelle de la scène) est faite par le pont.

## Fonctions

| Fonction | Rôle | Retour |
|---|---|---|
| `esquisse_depuis_contour(plan, origine, contour, nom=None)` | crée une esquisse à partir d'un contour | l'esquisse |
| `executer_gabarit(gabarit, parametres=None, piece=None)` | esquisses + recette OCCT + recalcul | `{piece, objet, esquisses, parametres}` |
| `mesurer(piece)` | dimensions au format `fiche.json` | `{dimensions_mm, boite_mm, volume_mm3}` |
| `evaluer(expr, parametres)` | expression arithmétique sûre | nombre |

Toute donnée invalide lève `api.ErreurPont`, avec un message lisible.

## Plans

| Al-Tasrif (VUES) | Sketcher |
|---|---|
| `dessus` | XY |
| `profil` | XZ |
| `face` | YZ |

`origine` = (u, v) en mm : décalage du contour dans le repère du plan.

## Contour

```json
{"construction": false, "segments": [
  {"type": "line",    "p1": [0, 0], "p2": [60, 0]},
  {"type": "arc",     "ct": [60, 4], "p1": [60, 0], "p2": [60, 8], "ccw": true},
  {"type": "bezier",  "p1": [60, 8], "c1": [36, 8], "c2": [18, 12], "p2": [0, 8]},
  {"type": "line",    "p1": [0, 8], "p2": [0, 0]},
  {"type": "circle",  "ct": [30, 4], "r": 1.5},
  {"type": "ellipse", "ct": [80, 0], "a": [10.375, 0], "b": [0, 9.125]}
]}
```

- Les extrémités distantes de moins de 0,0001 mm deviennent **un seul point** : le contour est connecté, donc extrudable.
- `arc` : `ccw: false` parcourt l'arc dans le sens horaire.
- `ellipse` : `a` et `b` sont les demi-axes donnés comme vecteurs depuis le centre.
- Chaque segment accepte `"construction": true`.

## Gabarit

```json
{
  "nom": "Mors",
  "parametres": {"L": {"defaut": 60}, "h": {"defaut": 8}, "e": {"defaut": 3}, "r": "h/2"},
  "esquisses": [
    {"nom": "S1", "plan": "dessus", "origine": [0, 0],
     "contour": {"segments": [{"type": "line", "p1": [0, 0], "p2": ["L", 0]}]}}
  ],
  "fonctions": [
    {"op": "pad", "sketch": "S1", "depth": "e"},
    {"op": "fillet", "select": "TOP", "radius": 0.5}
  ]
}
```

- Toute valeur numérique peut être une **expression** : `"L/2 + 3"`, `"e*cos(30)"`. Sont permis les paramètres, `+ - * / ** %`, `pi`, `sin cos tan` (en degrés), `sqrt abs min max`. Rien d'autre n'est exécuté.
- Un paramètre peut dépendre d'un autre (`"r": "h/2"`).
- `parametres` passé à `executer_gabarit` remplace les valeurs par défaut.
- Fonctions disponibles : `pad`, `pocket` (`sketch`, `depth`, `symmetric`) ; `fillet`, `chamfer` (`radius`, `select` : `ALL`, `TOP`, `BOTTOM`, `VERTICAL`, `CURSOR` + `probe`, `PICKED` + `probes`).
- Les esquisses sont nommées `<piece>.<nom>`, et la recette est enregistrée sous le nom de la pièce, qu'on peut donc recalculer.

## Mesure

```json
{"dimensions_mm": {"longueur": 63.999, "largeur": 9.778, "epaisseur": 3.0},
 "boite_mm": [63.999, 9.778, 3.0], "volume_mm3": 1680.688}
```

`dimensions_mm` suit le format de `fiche.json` : les côtés de la boîte englobante, triés du plus grand au plus petit. Al-Tasrif peut comparer directement avec la fiche (`controle.py`).

## Prochaines étapes

- Côté Al-Tasrif : exporter un contour depuis `vues.json` / `planches.json` (pixels → mm avec `mm_par_px` et `origine_mm`), puis bouton « Envoyer à Sketcher ».
- Côté Sketcher : contraintes et cotes pilotées depuis le gabarit, fonctions `pattern` et `mirror`.
