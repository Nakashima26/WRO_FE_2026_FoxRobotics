"""Posterior del sorteo de wro_field, condicionado a lo observado en la pista."""

from functools import lru_cache
from itertools import combinations, permutations

from .wro_field import (
    CARDS, CARD_SINGLE_GREEN, CARD_SINGLE_RED, PARKING_FORBIDDEN_SEATS,
    REQUIRED_CARDS, SEATS, SECTIONS_CW, _valid_zones,
)

_COLORS = ("Red", "Green")
_SLOTS = tuple(SEATS)
_WIDTH = len(_SLOTS) * len(_COLORS)
_COLUMN = {
    (i, seat, color): i * _WIDTH + j * len(_COLORS) + k
    for i in range(len(SECTIONS_CW))
    for j, seat in enumerate(_SLOTS)
    for k, color in enumerate(_COLORS)
}
_PARK_OK = {n for n, card in enumerate(CARDS, 1)
            if not set(dict(card)).intersection(PARKING_FORBIDDEN_SEATS)}


@lru_cache(maxsize=1)
def _deals():
    """Agrupa los repartos por sus cuatro cartas y su multiplicidad de sorteo."""
    weights = {}
    for single in (CARD_SINGLE_GREEN, CARD_SINGLE_RED):
        for required in REQUIRED_CARDS:
            pool = set(range(1, len(CARDS) + 1)) - {single, required}
            for others in combinations(sorted(pool), 2):
                cards = tuple(sorted((single, required, *others)))
                colors = [color for n in cards for _, color in CARDS[n - 1]]
                if len(colors) >= 5 and abs(colors.count("Red") - colors.count("Green")) <= 1:
                    # El orden de las dos cartas extra aporta un factor común de 2.
                    weights[cards] = weights.get(cards, 0) + 1
    return weights


@lru_cache(maxsize=8)
def _table(direction, start):
    """Índices de bits por color/asiento y por peso, con el cajón en la recta 0."""
    zones = {n: _valid_zones(n, direction) for n in range(1, len(CARDS) + 1)}
    eligible = _PARK_OK
    if start == "zona":
        eligible = {n for n in eligible if zones[n]}
    elif start in ("Z3", "Z4"):
        eligible = {n for n in eligible if start in zones[n]}

    rows = []
    for cards, multiplicity in _deals().items():
        if not any(zones[n] for n in cards):
            continue
        parking = eligible.intersection(cards)
        for parked in sorted(parking):
            # randomize elige uniformemente entre las rectas aptas para el cajón.
            weight = 12 * multiplicity // len(parking)
            for other in permutations(n for n in cards if n != parked):
                rows.append(((parked, *other), weight))

    size = (len(rows) + 7) // 8
    columns = [bytearray(size) for _ in _COLUMN]
    weights = {weight: bytearray(size) for _, weight in rows}
    for index, (cards, weight) in enumerate(rows):
        byte, bit = index // 8, 1 << (index % 8)
        weights[weight][byte] |= bit
        for i, number in enumerate(cards):
            for seat, color in CARDS[number - 1]:
                columns[_COLUMN[i, seat, color]][byte] |= bit
    return (1 << len(rows)) - 1, tuple(int.from_bytes(c, "little") for c in columns), tuple(
        (weight, int.from_bytes(bits, "little")) for weight, bits in weights.items()
    )


def prepare(direction, start="cajon"):
    """Precalienta el índice si el sentido ya se conoce."""
    if start == "centro":
        start = "cajon"
    _table(direction, start)


@lru_cache(maxsize=128)
def _counts(direction, start, confirmed, empty):
    mask, columns, weights = _table(direction, start)
    for column in confirmed:
        mask &= columns[column]
    for red, green in empty:
        mask &= ~(columns[red] | columns[green])
    if not mask:
        return 0, ()

    def mass(bits):
        return sum(weight * (bits & group).bit_count() for weight, group in weights)

    return mass(mask), tuple(mass(mask & col) for col in columns)


def posterior(direction, parking, confirmed, empty, start="cajon"):
    """Dict por asiento pendiente con (P(señal), P(roja), P(verde)).

    None indica observaciones incompatibles. start describe cómo se solicitó
    el sorteo, igual que en randomize. Un confirmado prevalece sobre un vacío.
    """
    if direction not in ("CW", "CCW") or parking not in SECTIONS_CW:
        return None
    if start == "centro":
        start = "cajon"

    offset = SECTIONS_CW.index(parking)
    seat_columns = {
        (SECTIONS_CW[(offset + i) % len(SECTIONS_CW)], seat):
        (_COLUMN[i, seat, "Red"], _COLUMN[i, seat, "Green"])
        for i in range(len(SECTIONS_CW)) for seat in _SLOTS
    }
    if any(key not in seat_columns or color not in _COLORS for key, color in confirmed.items()):
        return None
    empty = set(empty).difference(confirmed)
    total, counts = _counts(
        direction, start,
        tuple(sorted(seat_columns[key][_COLORS.index(color)] for key, color in confirmed.items())),
        tuple(sorted(seat_columns[key] for key in empty if key in seat_columns)),
    )
    if not total:
        return None
    return {
        key: ((counts[r] + counts[g]) / total, counts[r] / total, counts[g] / total)
        for key, (r, g) in seat_columns.items() if key not in confirmed and key not in empty
    }
