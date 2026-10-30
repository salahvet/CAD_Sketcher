"""Pont Al-Tasrif -> CAD Sketcher.

Repartition des roles :
    Al-Tasrif  = references, donnees de modelisation, gabarits (stockage)
    Sketcher   = execution : esquisses, contraintes, fonctions OCCT

Ce module est l'unique point d'entree public. Il ne depend que de bpy et
de Sketcher ; Al-Tasrif l'appelle sans connaitre les internes :

    import importlib
    api = importlib.import_module("bl_ext.user_default.CAD_Sketcher.api")
    if api.API_VERSION[0] == 1:
        sk = api.esquisse_depuis_contour("dessus", (0, 0), contour)

Toutes les longueurs echangees sont en millimetres. Les formats sont
decrits dans docs/api_pont.md.
"""

import ast
import math
import operator

import bpy
from mathutils import Vector

API_VERSION = (1, 3)

# plans d'Al-Tasrif (VUES) -> plans d'origine de Sketcher
PLANS = {
    "dessus": "origin_plane_XY", "xy": "origin_plane_XY",
    "profil": "origin_plane_XZ", "xz": "origin_plane_XZ",
    "face": "origin_plane_YZ", "yz": "origin_plane_YZ",
}

TOLERANCE_MM = 1e-4  # extremites plus proches = un seul point (coincidence)


class ErreurPont(Exception):
    """Donnee invalide transmise par l'appelant."""


# ---------------------------------------------------------------------------
# Unites
# ---------------------------------------------------------------------------

def _mm_par_unite(context):
    """Millimetres par unite Blender (1000 avec l'echelle par defaut)."""
    return 1000.0 * (context.scene.unit_settings.scale_length or 1.0)


# ---------------------------------------------------------------------------
# Expressions de gabarit : arithmetique seule, aucun appel arbitraire
# ---------------------------------------------------------------------------

_OPS = {
    ast.Add: operator.add, ast.Sub: operator.sub, ast.Mult: operator.mul,
    ast.Div: operator.truediv, ast.Pow: operator.pow, ast.Mod: operator.mod,
    ast.USub: operator.neg, ast.UAdd: operator.pos,
}
_FONCS = {
    "sin": lambda d: math.sin(math.radians(d)),
    "cos": lambda d: math.cos(math.radians(d)),
    "tan": lambda d: math.tan(math.radians(d)),
    "sqrt": math.sqrt, "abs": abs, "min": min, "max": max,
}


def evaluer(expr, parametres):
    """Evalue un nombre ou une expression ("L/2 + 3", "e*cos(30)").

    Angles en degres. Seuls les parametres, + - * / ** %, et les fonctions
    de _FONCS sont permis.
    """
    if isinstance(expr, (int, float)):
        return float(expr)
    if not isinstance(expr, str):
        raise ErreurPont(f"unexpected value: {expr!r}")

    def ev(n):
        if isinstance(n, ast.Expression):
            return ev(n.body)
        if isinstance(n, ast.Constant) and isinstance(n.value, (int, float)):
            return float(n.value)
        if isinstance(n, ast.Name):
            if n.id in parametres:
                return float(parametres[n.id])
            if n.id == "pi":
                return math.pi
            raise ErreurPont(f"unknown parameter: {n.id}")
        if isinstance(n, ast.BinOp) and type(n.op) in _OPS:
            return _OPS[type(n.op)](ev(n.left), ev(n.right))
        if isinstance(n, ast.UnaryOp) and type(n.op) in _OPS:
            return _OPS[type(n.op)](ev(n.operand))
        if (isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
                and n.func.id in _FONCS and not n.keywords):
            return float(_FONCS[n.func.id](*[ev(a) for a in n.args]))
        raise ErreurPont(f"expression not allowed: {expr!r}")

    try:
        return ev(ast.parse(expr, mode="eval"))
    except SyntaxError as e:
        raise ErreurPont(f"invalid expression: {expr!r}") from e


def _substituer(obj, parametres):
    """Remplace recursivement les expressions par leur valeur."""
    if isinstance(obj, dict):
        return {k: (v if k in ("type", "nom", "op", "sketch", "select", "plan", "kind", "direction", "axis")
                    else _substituer(v, parametres)) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_substituer(v, parametres) for v in obj]
    if isinstance(obj, str):
        return evaluer(obj, parametres)
    return obj


# ---------------------------------------------------------------------------
# Esquisse depuis un contour
# ---------------------------------------------------------------------------

def _plan(context, plan):
    sse = context.scene.sketcher.entities
    attr = PLANS.get(str(plan).lower())
    if attr is None:
        raise ErreurPont(f"unknown plane: {plan!r} (expected one of {sorted(set(PLANS))})")
    sse.ensure_origin_elements(context)
    wp = getattr(sse, attr)
    if wp is None:
        raise ErreurPont(f"origin plane missing: {attr}")
    return wp.slvs_index


def esquisse_depuis_contour(plan, origine, contour, nom=None, context=None):
    """Cree une esquisse Sketcher depuis un contour Al-Tasrif.

    plan     : "dessus" | "profil" | "face" (ou "XY", "XZ", "YZ")
    origine  : (u, v) en mm, decalage du contour dans le repere du plan
    contour  : {"segments": [...], "construction": False} (voir docs)
    nom      : nom de l'esquisse (sert de reference aux fonctions)

    Les extremites communes deviennent un meme point : le contour est
    connecte, et donc fermable/extrudable. Retourne l'esquisse.
    """
    context = context or bpy.context
    sse = context.scene.sketcher.entities
    k = 1.0 / _mm_par_unite(context)
    ou, ov = (float(origine[0]), float(origine[1])) if origine else (0.0, 0.0)
    segments = contour.get("segments") if isinstance(contour, dict) else contour
    if not segments:
        raise ErreurPont("empty contour")
    construction = bool(contour.get("construction")) if isinstance(contour, dict) else False

    wp_i = _plan(context, plan)
    sk = sse.add_sketch(sse.get(wp_i))
    sk_i = sk.slvs_index
    if nom:
        sk.name = nom

    # ATTENTION : chaque ajout peut reallouer les collections ; on ne garde
    # que des index et on relit avec get() juste avant usage.
    get = sse.get
    connus = []  # [(Vector mm, index)]

    def point(co, partage=True):
        v = Vector((float(co[0]) + ou, float(co[1]) + ov))
        if partage:
            for c, i in connus:
                if (c - v).length <= TOLERANCE_MM:
                    return i
        i = sse.add_point_2d(v * k, get(sk_i), index_reference=True)
        if partage:
            connus.append((v, i))
        return i

    crees = []
    for n, seg in enumerate(segments):
        t = seg.get("type")
        cons = bool(seg.get("construction", construction))
        try:
            if t == "line":
                e = sse.add_line_2d(get(point(seg["p1"])), get(point(seg["p2"])), get(sk_i))
            elif t == "arc":
                p1, p2 = point(seg["p1"]), point(seg["p2"])
                if not seg.get("ccw", True):
                    p1, p2 = p2, p1  # meme arc parcouru dans l'autre sens
                ct = point(seg["ct"], partage=False)
                sk = get(sk_i)
                e = sse.add_arc(sk.wp.nm, get(ct), get(p1), get(p2), sk)
            elif t == "circle":
                ct = point(seg["ct"], partage=False)
                sk = get(sk_i)
                e = sse.add_circle(sk.wp.nm, get(ct), float(seg["r"]) * k, sk)
            elif t == "bezier":
                p1, p2 = point(seg["p1"]), point(seg["p2"])
                c1, c2 = point(seg["c1"], partage=False), point(seg["c2"], partage=False)
                e = sse.add_bezier(p1, c1, c2, p2, get(sk_i))
            elif t == "ellipse":
                ct = Vector(seg["ct"])
                a = ct + Vector(seg["a"])  # demi-axes donnes comme vecteurs
                b = ct + Vector(seg["b"])
                e = sse.add_ellipse(get(point(seg["ct"], False)), get(point(a, False)),
                                    get(point(b, False)), get(sk_i))
            else:
                raise ErreurPont(f"segment {n}: unknown type {t!r}")
        except KeyError as err:
            raise ErreurPont(f"segment {n} ({t}): missing field {err}") from None
        e.construction = cons
        crees.append(e.slvs_index)

    sk = get(sk_i)
    sk.solve(context)
    return sk


# ---------------------------------------------------------------------------
# Gabarits -> esquisses + recette OCCT
# ---------------------------------------------------------------------------

def _parametres_effectifs(gabarit, parametres):
    vals = {}
    for nom, dfn in (gabarit.get("parametres") or {}).items():
        vals[nom] = dfn.get("defaut") if isinstance(dfn, dict) else dfn
    vals.update(parametres or {})
    # un parametre peut dependre d'un autre : plusieurs passes
    for _ in range(len(vals) + 1):
        stable = True
        for nom, v in list(vals.items()):
            if isinstance(v, str):
                try:
                    vals[nom] = evaluer(v, {k: x for k, x in vals.items()
                                            if not isinstance(x, str)})
                    stable = False
                except ErreurPont:
                    pass
        if stable:
            break
    restants = [n for n, v in vals.items() if isinstance(v, str) or v is None]
    if restants:
        raise ErreurPont(f"parameters without a value: {restants}")
    return vals


def executer_gabarit(gabarit, parametres=None, piece=None, context=None):
    """Construit une piece depuis un gabarit Al-Tasrif.

    gabarit    : {"nom", "parametres", "esquisses": [...], "fonctions": [...]}
    parametres : valeurs qui remplacent les defauts du gabarit (mm, degres)
    piece      : nom de l'objet produit (defaut : gabarit["nom"])

    Les esquisses sont creees, puis la recette OCCT de la piece est ecrite
    et recalculee. Retourne {"piece", "objet", "esquisses", "parametres"}.
    """
    from .operators import occt_ops

    context = context or bpy.context
    vals = _parametres_effectifs(gabarit, parametres)
    piece = piece or gabarit.get("nom") or "Piece"
    k = 1.0 / _mm_par_unite(context)

    noms = {}
    for i, esq in enumerate(gabarit.get("esquisses") or []):
        esq = _substituer(esq, vals)
        nom = f"{piece}.{esq.get('nom') or 'S%d' % (i + 1)}"
        esquisse_depuis_contour(esq.get("plan", "dessus"), esq.get("origine"),
                                esq["contour"], nom=nom, context=context)
        noms[esq.get("nom")] = nom

    recette = []
    for f in gabarit.get("fonctions") or []:
        f = _substituer(f, vals)
        op = f.get("op")
        if op not in ("pad", "pocket", "fillet", "chamfer", "pattern"):
            raise ErreurPont(f"unsupported feature: {op!r}")
        f = dict(f)
        if "sketch" in f:
            if f["sketch"] not in noms:
                raise ErreurPont(f"unknown sketch: {f['sketch']!r}")
            f["sketch"] = noms[f["sketch"]]
        for cle in ("depth", "radius", "spacing"):  # mm -> unites Blender
            if cle in f:
                f[cle] = float(f[cle]) * k
        if op == "pattern":
            # « feature » : numero (1, 2, ...) de la fonction repetee dans la liste du gabarit
            n = int(f.get("feature", len(recette)))
            if not 1 <= n <= len(recette) or recette[n - 1]["op"] not in ("pad", "pocket"):
                raise ErreurPont(f"pattern: feature {n} is not a pad or a pocket")
            f["feature"] = n - 1
            f["count"] = int(round(float(f.get("count", 2))))
            if "center" in f:
                f["center"] = [float(c) * k for c in f["center"]]
        recette.append(f)

    recettes = occt_ops.get_recipes(context.scene)
    recettes[piece] = recette
    occt_ops.set_recipes(context.scene, recettes)
    obj = occt_ops.recompute_part(context, piece) if recette else None
    return {"piece": piece, "objet": obj, "esquisses": list(noms.values()),
            "parametres": vals}


# ---------------------------------------------------------------------------
# Mesure : retour vers Al-Tasrif (controle contre fiche.json)
# ---------------------------------------------------------------------------

def mesurer(piece, context=None):
    """Dimensions d'une piece, au format de fiche.json d'Al-Tasrif.

    Retourne {"dimensions_mm": {"longueur", "largeur", "epaisseur"},
              "boite_mm": [x, y, z], "volume_mm3": float | None}
    longueur >= largeur >= epaisseur (cotes de la boite englobante triees).
    """
    from .operators import occt_ops

    context = context or bpy.context
    obj = context.scene.objects.get(piece) if isinstance(piece, str) else piece
    if obj is None:
        raise ErreurPont(f"part not found: {piece!r}")
    mm = _mm_par_unite(context)
    coins = [obj.matrix_world @ Vector(c) for c in obj.bound_box]
    boite = [(max(c[a] for c in coins) - min(c[a] for c in coins)) * mm for a in range(3)]
    tri = sorted(boite, reverse=True)

    volume = None
    solide = occt_ops._solid_cache.get(obj.name)
    if solide is not None:
        try:
            volume = float(solide.volume) * mm ** 3
        except Exception:
            volume = None
    r = lambda x: round(x, 3)
    return {
        "dimensions_mm": {"longueur": r(tri[0]), "largeur": r(tri[1]), "epaisseur": r(tri[2])},
        "boite_mm": [r(x) for x in boite],
        "volume_mm3": None if volume is None else r(volume),
    }


# ---------------------------------------------------------------------------
# Automatisation sur un dessin existant (v1.1)
# ---------------------------------------------------------------------------

def _esquisse(esquisse, context):
    if isinstance(esquisse, str):
        for s in context.scene.sketcher.entities.sketches:
            if s.name == esquisse:
                return s
        raise ErreurPont(f"sketch not found: {esquisse!r}")
    return esquisse


def contraintes_essentielles(esquisse, angle_tangence=8.0, angle_axe=2.0, context=None):
    """Tangences aux jonctions lisses, horizontal / vertical, contacts ; retire les redondances.

    Ne crée aucune géométrie. Retourne le bilan {nature: [détails]}.
    """
    from .operators.auto_constraints import auto_constraints as f
    context = context or bpy.context
    return f(context, _esquisse(esquisse, context), angle_tangence, angle_axe)


def cotes_proposees(esquisse, pas_mm=0.05, context=None):
    """Cotes de base avec les valeurs du dessin arrondies au pas. Retourne (bilan, etat, ddl)."""
    from .operators.auto_constraints import propose_dimensions as f
    context = context or bpy.context
    return f(context, _esquisse(esquisse, context), pas_mm)


def contraindre_totalement(esquisse, pas_mm=0.05, poignees=True, context=None):
    """Contraint l'esquisse jusqu'a 0 degre de liberte, aux valeurs du dessin (v1.2).

    Retourne (bilan, etat, ddl).
    """
    from .operators.auto_constraints import fully_constrain as f
    context = context or bpy.context
    return f(context, _esquisse(esquisse, context), pas_mm, poignees)


__all__ = ["API_VERSION", "PLANS", "ErreurPont", "evaluer",
           "esquisse_depuis_contour", "executer_gabarit", "mesurer",
           "contraintes_essentielles", "cotes_proposees", "contraindre_totalement"]
