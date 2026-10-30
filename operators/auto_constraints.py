"""Fork : « Contraintes essentielles » — contraintes déduites du dessin de l'utilisateur.

L'utilisateur dessine ; cet outil ne crée aucune géométrie. Il lit l'esquisse et ajoute
seulement ce que le dessin montre déjà :
  - tangence aux jonctions lisses (lignes, arcs, Béziers qui se suivent sans angle) ;
  - horizontal / vertical pour les lignes presque alignées sur les axes ;
  - coïncidence des extrémités qui se touchent sans être liées ;
et retire les contraintes redondantes (un point contraint sur sa propre extrémité).
Les cotes (distances, rayons, angles) restent à l'utilisateur : ce sont ses choix.
"""
import math
from collections import defaultdict

from bpy.props import BoolProperty, FloatProperty
from bpy.types import Context, Operator
from bpy.utils import register_classes_factory

from ..declarations import Operators
from ..model.types import SlvsArc, SlvsBezier, SlvsEllipse, SlvsLine2D
from ..utilities.view import refresh


def _direction_sortante(e, pt):
    """Direction (unitaire) par laquelle la courbe e quitte son extrémité pt, ou None."""
    co = pt.co
    if isinstance(e, SlvsLine2D):
        autre = e.p2 if pt == e.p1 else e.p1
        v = autre.co - co
    elif isinstance(e, SlvsBezier):
        ctrl = e.ctrl1 if pt == e.p1 else e.ctrl2
        v = ctrl.co - co
        if v.length < 1e-12:  # poignée confondue : vers l'autre poignée
            v = (e.ctrl2 if pt == e.p1 else e.ctrl1).co - co
    else:
        return None
    return v.normalized() if v.length > 1e-12 else None


def _milieu_arc(arc):
    """Point au milieu de l'arc, pour orienter sa tangente."""
    c = arc.ct.co
    a1 = math.atan2(*(arc.p1.co - c)[::-1])
    a2 = math.atan2(*(arc.p2.co - c)[::-1])
    sweep = (a2 - a1) % (2 * math.pi)
    if getattr(arc, "invert_direction", False):
        sweep -= 2 * math.pi
    am = a1 + sweep / 2
    r = (arc.p1.co - c).length
    return c + c.__class__((math.cos(am), math.sin(am))) * r


def _sortante(e, pt):
    if isinstance(e, SlvsArc):
        v = (pt.co - e.ct.co).orthogonal()
        if v.dot(_milieu_arc(e) - pt.co) < 0:
            v = -v
        return v.normalized() if v.length > 1e-12 else None
    return _direction_sortante(e, pt)


def _deja(constraints, type_, a, b=None):
    ia = a.slvs_index
    ib = b.slvs_index if b is not None else None
    for c in constraints.get_list(type_):
        idx = [x.slvs_index for x in c.entities()]
        if ib is None and ia in idx:
            return True
        if ib is not None and ia in idx and ib in idx:
            return True
    return False


def contraintes_essentielles(context, sketch, angle_tangence=8.0, angle_axe=2.0,
                             tolerance_contact=1e-5, tangence=True, axes=True,
                             contacts=True, nettoyer=True, dry_run=False):
    """Ajoute à `sketch` les contraintes essentielles. Retourne un bilan {nature: [détails]}.

    angle_tangence : écart max (degrés) pour qu'une jonction soit jugée lisse
    angle_axe      : écart max (degrés) d'une ligne à l'horizontale / la verticale
    tolerance_contact : distance (unités Blender) sous laquelle deux extrémités se touchent
    dry_run        : ne rien modifier, seulement rapporter
    """
    S = context.scene.sketcher
    sse = S.entities
    cons = S.constraints
    bilan = defaultdict(list)
    sk_i = sketch.slvs_index

    segs = [e for e in sse.all
            if getattr(e, "sketch", None) == sketch and not e.is_point()
            and isinstance(e, (SlvsLine2D, SlvsBezier, SlvsArc)) and not e.construction]

    # 1. redondances : un point contraint coïncident avec une entité dont il est l'extrémité
    if nettoyer:
        a_retirer = []
        for c in cons.coincident:
            if c.sketch != sketch:
                continue
            a, b = c.entity1, c.entity2
            pt, ent = (a, b) if a.is_point() else (b, a)
            if pt.is_point() and not ent.is_point() and pt in getattr(ent, "connection_points", lambda: [])():
                a_retirer.append(c)
        bilan["redondantes retirées"] = [f"{c.entity1.name} / {c.entity2.name}" for c in a_retirer]
        if not dry_run:
            for c in a_retirer:
                cons.remove(c)

    # 2. contacts : extrémités distinctes au même endroit
    extremites = {}
    for e in segs:
        for p in e.connection_points():
            extremites[p.slvs_index] = p
    if contacts:
        pts = list(extremites.values())
        for i, p in enumerate(pts):
            for q in pts[i + 1:]:
                if (p.co - q.co).length <= tolerance_contact and not _deja(cons, "COINCIDENT", p, q):
                    bilan["coïncidences"].append(f"{p.name} = {q.name}")
                    if not dry_run:
                        cons.add_coincident(p.slvs_index, q.slvs_index, sketch=sk_i)

    # 3. tangences aux jonctions lisses
    if tangence:
        par_point = defaultdict(list)
        for e in segs:
            for p in e.connection_points():
                par_point[p.slvs_index].append(e)
        for pi, ents in par_point.items():
            if len(ents) != 2:
                continue
            e1, e2 = ents
            if isinstance(e1, SlvsLine2D) and isinstance(e2, SlvsLine2D):
                continue  # deux lignes ne sont jamais « tangentes » : angle ou colinéaires
            pt = extremites[pi]
            d1, d2 = _sortante(e1, pt), _sortante(e2, pt)
            if d1 is None or d2 is None:
                continue
            ecart = math.degrees(math.acos(max(-1.0, min(1.0, -d1.dot(d2)))))
            if ecart > angle_tangence:
                bilan["angles laissés vifs"].append(f"{pt.name} ({ecart:.0f}°)")
                continue
            # la contrainte veut une courbe en premier
            if isinstance(e1, SlvsLine2D):
                e1, e2 = e2, e1
            if _deja(cons, "TANGENT", e1, e2):
                continue
            bilan["tangences"].append(f"{e1.name} ~ {e2.name} ({ecart:.1f}°)")
            if not dry_run:
                cons.add_tangent(e1.slvs_index, e2.slvs_index, sketch=sk_i)

    # 4. lignes horizontales / verticales
    if axes:
        for e in segs:
            if not isinstance(e, SlvsLine2D):
                continue
            v = e.p2.co - e.p1.co
            if v.length < 1e-9 or _deja(cons, "HORIZONTAL", e) or _deja(cons, "VERTICAL", e):
                continue
            a = abs(math.degrees(math.atan2(v.y, v.x))) % 180
            if min(a, 180 - a) <= angle_axe:
                bilan["horizontales"].append(e.name)
                if not dry_run:
                    cons.add_horizontal(e.slvs_index, sketch=sk_i)
            elif abs(a - 90) <= angle_axe:
                bilan["verticales"].append(e.name)
                if not dry_run:
                    cons.add_vertical(e.slvs_index, sketch=sk_i)

    if not dry_run:
        sketch = sse.get(sk_i)
        sketch.solve(context)
        bilan["_etat"] = [sketch.solver_state, getattr(sketch, "dof", None)]
    return dict(bilan)


# ---------------------------------------------------------------------------
# Cotes proposées
# ---------------------------------------------------------------------------

def _etat(context, sketch_i):
    sk = context.scene.sketcher.entities.get(sketch_i)
    sk.solve(context)
    return sk.solver_state, getattr(sk, "dof", None)


def cotes_proposees(context, sketch, pas_mm=0.05, longueurs=True, ellipses=True,
                    position=True, encombrement=True):
    """Pose les cotes de base avec les valeurs du dessin, arrondies au pas (mm).

    Chaque cote est essayée seule : si le solveur la refuse (sur-contrainte,
    redondance), elle est retirée. Retourne {nature: [libellés]} et l'état final.
    - longueurs : longueur de chaque ligne
    - ellipses  : les deux demi-axes, et la position du centre
    - position  : le point le plus proche de l'origine, coté depuis un point
                  fixe « Origine » (construction) : le dessin est ancré
    - encombrement : distance horizontale origine -> extrémités gauche et droite
    Les Béziers gardent leurs poignées libres : la forme reste à l'utilisateur.
    """
    S = context.scene.sketcher
    sse, cons = S.entities, S.constraints
    sk_i = sketch.slvs_index
    mm = 1000.0 * (context.scene.unit_settings.scale_length or 1.0)
    bilan = defaultdict(list)
    etat0, dof0 = _etat(context, sk_i)

    def arrondi(v):
        return round(v * mm / pas_mm) * pas_mm / mm

    def essayer(libelle, e1, e2=None, align="NONE"):
        e1_i = e1 if isinstance(e1, int) else e1.slvs_index
        e2_i = None if e2 is None else (e2 if isinstance(e2, int) else e2.slvs_index)
        avant = _etat(context, sk_i)
        c = cons.add_distance(e1_i, e2_i, sketch=sk_i, init=True,
                              **({"align": align} if align != "NONE" else {}))
        c.value = arrondi(c.value)
        etat, dof = _etat(context, sk_i)
        if etat != "OKAY" or (avant[1] is not None and dof is not None and dof >= avant[1]):
            cons.remove(c)
            _etat(context, sk_i)
            bilan["refusées"].append(libelle)
            return False
        bilan["posées"].append(f"{libelle} = {c.value * mm:g} mm")
        return True

    if etat0 != "OKAY":
        return {"refusées": [f"esquisse pas résolue ({etat0}) : lancer d'abord Contraintes essentielles"]}, etat0, dof0

    def segs():
        sk = sse.get(sk_i)
        return [e for e in sse.all if getattr(e, "sketch", None) == sk and not e.is_point()
                and not e.construction]

    # point fixe à l'origine (le pivot des planches Al-Tasrif est à l'origine)
    origine_i = None
    if position or encombrement:
        sk = sse.get(sk_i)
        for p in sse.points2D:
            if p.sketch == sk and p.fixed and p.co.length < 1e-9:
                origine_i = p.slvs_index
                break
        if origine_i is None:
            p = sse.add_point_2d((0.0, 0.0), sse.get(sk_i), fixed=True, construction=True)
            p.name = "Origine"
            origine_i = p.slvs_index
            _etat(context, sk_i)

    if longueurs:
        for e in [e.slvs_index for e in segs() if isinstance(e, SlvsLine2D)]:
            essayer(f"longueur {sse.get(e).name}", e)

    if ellipses:
        for e_i in [e.slvs_index for e in segs() if isinstance(e, SlvsEllipse)]:
            el = sse.get(e_i)
            ct, pa, pb = el.ct.slvs_index, el.pa.slvs_index, el.pb.slvs_index
            essayer(f"{el.name} demi-axe a", ct, pa)
            essayer(f"{el.name} demi-axe b", ct, pb)
            if origine_i is not None:
                essayer(f"{el.name} centre (horizontal)", origine_i, ct, "HORIZONTAL")
                essayer(f"{el.name} centre (vertical)", origine_i, ct, "VERTICAL")

    if origine_i is not None:
        extremites = {}
        for e in segs():
            if isinstance(e, SlvsEllipse):
                continue
            for p in e.connection_points():
                extremites[p.slvs_index] = p.co.copy()
        if extremites and position:
            proche = min(extremites, key=lambda i: extremites[i].length)
            n = sse.get(proche).name
            essayer(f"position {n} (horizontal)", origine_i, proche, "HORIZONTAL")
            essayer(f"position {n} (vertical)", origine_i, proche, "VERTICAL")
        if extremites and encombrement:
            for i in {min(extremites, key=lambda i: extremites[i].x),
                      max(extremites, key=lambda i: extremites[i].x)}:
                essayer(f"encombrement {sse.get(i).name}", origine_i, i, "HORIZONTAL")

    etat, dof = _etat(context, sk_i)
    return dict(bilan), etat, dof


class View3D_OT_slvs_auto_dimensions(Operator):
    """Pose les cotes de base avec les valeurs de votre dessin (arrondies) : longueurs des lignes,
demi-axes et centre des ellipses, ancrage sur l'origine, encombrement. Une cote refusée par
le solveur est retirée. Modifiez ensuite les valeurs : le dessin suit"""

    bl_idname = Operators.AutoDimensions
    bl_label = "Cotes proposées"
    bl_options = {"REGISTER", "UNDO"}

    pas_mm: FloatProperty(name="Arrondi (mm)", default=0.05, min=0.001, max=5.0)
    longueurs: BoolProperty(name="Longueurs des lignes", default=True)
    ellipses: BoolProperty(name="Ellipses", default=True)
    position: BoolProperty(name="Ancrage sur l'origine", default=True)
    encombrement: BoolProperty(name="Encombrement", default=True)

    @classmethod
    def poll(cls, context):
        return context.scene.sketcher.active_sketch is not None

    def execute(self, context: Context):
        sketch = context.scene.sketcher.active_sketch
        bilan, etat, dof = cotes_proposees(context, sketch, self.pas_mm, self.longueurs,
                                           self.ellipses, self.position, self.encombrement)
        poses, refus = len(bilan.get("posées", [])), len(bilan.get("refusées", []))
        self.report({"INFO" if etat == "OKAY" else "WARNING"},
                    f"{poses} cote(s) posée(s), {refus} refusée(s) ; solveur {etat}, {dof} degré(s) de liberté")
        refresh(context)
        return {"FINISHED"}


class View3D_OT_slvs_auto_constraints(Operator):
    """Ajoute les contraintes que le dessin montre déjà : tangences aux jonctions lisses,
lignes horizontales / verticales, extrémités en contact ; retire les redondances.
Ne crée aucune géométrie et ne pose aucune cote"""

    bl_idname = Operators.AutoConstraints
    bl_label = "Contraintes essentielles"
    bl_options = {"REGISTER", "UNDO"}

    angle_tangence: FloatProperty(name="Jonction lisse sous", default=8.0, min=0.5, max=30.0,
                                  description="Écart (degrés) en dessous duquel une jonction devient tangente")
    angle_axe: FloatProperty(name="Ligne d'axe sous", default=2.0, min=0.0, max=10.0,
                             description="Écart (degrés) en dessous duquel une ligne devient horizontale / verticale")
    tangence: BoolProperty(name="Tangences", default=True)
    axes: BoolProperty(name="Horizontal / vertical", default=True)
    contacts: BoolProperty(name="Extrémités en contact", default=True)
    nettoyer: BoolProperty(name="Retirer les redondances", default=True)

    @classmethod
    def poll(cls, context):
        return context.scene.sketcher.active_sketch is not None

    def execute(self, context: Context):
        sketch = context.scene.sketcher.active_sketch
        bilan = contraintes_essentielles(
            context, sketch, self.angle_tangence, self.angle_axe,
            tangence=self.tangence, axes=self.axes, contacts=self.contacts, nettoyer=self.nettoyer,
        )
        etat, dof = bilan.pop("_etat", (None, None))
        resume = ", ".join(f"{len(v)} {k}" for k, v in bilan.items() if v and k != "angles laissés vifs")
        vifs = len(bilan.get("angles laissés vifs", []))
        self.report({"INFO" if etat in ("OKAY", None) else "WARNING"},
                    f"{resume or 'rien à ajouter'} ; {vifs} angle(s) laissé(s) vif(s) ; "
                    f"solveur {etat}, {dof} degré(s) de liberté")
        refresh(context)
        return {"FINISHED"}


register, unregister = register_classes_factory(
    (View3D_OT_slvs_auto_constraints, View3D_OT_slvs_auto_dimensions)
)
