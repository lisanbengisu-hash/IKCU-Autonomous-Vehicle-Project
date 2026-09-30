# ============================================================
# SPECIAL ROUTES
# ============================================================
#
# Ozel rota mantigi.
#
# Simdilik sadece GYM hedefi icin ozel kural vardir:
#
# 1) Baslangic = anagiris:
#       anagiris -> gym
#
# 2) Baslangic = anacikia:
#       anacikia -> gym
#
# 3) Diger tum duraklar:
#       baslangic -> anacikia -> gym
#
# Iki A* parcasi kullanilsa bile controller'a TEK route listesi
# verilir. Kullanici ara duragi ayri bir hedef olarak gormez.
#
# Diger tum hedefler normal A* kullanir.
#
# Not:
# main_final.py gercek spawn konumunu start_location olarak
# verebilir. Bu, anacikia/anagiris gibi duraktan uzaga spawn
# olan noktalar icin daha guvenlidir.
# ============================================================


def merge_routes(*routes):
    """
    Birden fazla route listesini tek liste halinde birlestirir.

    Her yeni route'un ilk noktasi, onceki route'un son noktasi
    ile ayni ara hedefe ait oldugu icin ilk nokta tekrar eklenmez.
    """

    merged = []

    for route in routes:

        if route is None:
            continue

        if len(route) == 0:
            continue

        if not merged:
            merged.extend(route)
        else:
            merged.extend(route[1:])

    return merged


# ============================================================
# NORMAL ROUTE
# ============================================================

def plan_normal_route(
    planner,
    start_location,
    goal_location,
):
    return planner.plan(
        start_location,
        goal_location,
    )


# ============================================================
# GYM SPECIAL ROUTE
# ============================================================

def plan_gym_route(
    planner,
    bus_stops,
    start_name,
    start_location=None,
):
    """
    Gym icin ozel rota uretir.

    start_location verilirse gercek arac spawn konumu kullanilir.
    Verilmezse secilen duragin kayitli konumu kullanilir.
    """

    if start_location is None:
        start_location = (
            bus_stops[
                start_name
            ].location
        )

    exit_location = (
        bus_stops[
            "anacikia"
        ].location
    )

    gym_location = (
        bus_stops[
            "gym"
        ].location
    )

    print("\n" + "=" * 70)
    print("GYM OZEL ROTA")
    print("=" * 70)

    print(
        "Baslangic:",
        start_name,
    )

    # ========================================================
    # ANA GIRIS -> GYM
    # ========================================================

    if start_name == "anagiris":

        print("Rota:")
        print("anagiris -> gym")
        print("Ana Giris icin direkt Gym rotasi kullaniliyor.")

        return planner.plan(
            start_location,
            gym_location,
        )

    # ========================================================
    # ANA CIKIS -> GYM
    # ========================================================

    if start_name == "anacikia":

        print("Rota:")
        print("anacikia -> gym")
        print("Baslangic zaten Ana Cikis oldugu icin direkt rota kullaniliyor.")

        return planner.plan(
            start_location,
            gym_location,
        )

    # ========================================================
    # DIGER DURAKLAR -> ANA CIKIS -> GYM
    # ========================================================

    print("Rota:")
    print(
        f"{start_name}"
        " -> anacikia"
        " -> gym"
    )

    print(
        "\n1. A*:"
        f" {start_name} -> anacikia"
    )

    route_1 = planner.plan(
        start_location,
        exit_location,
    )

    print(
        "\n2. A*:"
        " anacikia -> gym"
    )

    route_2 = planner.plan(
        exit_location,
        gym_location,
    )

    route = merge_routes(
        route_1,
        route_2,
    )

    print(
        "\nGYM rotalari tek rota olarak birlestirildi."
    )

    print(
        "1. rota point:",
        len(route_1),
    )

    print(
        "2. rota point:",
        len(route_2),
    )

    print(
        "Birlesik route point:",
        len(route),
    )

    return route


# ============================================================
# ANA ROUTE FONKSIYONU
# ============================================================

def plan_special_route(
    planner,
    bus_stops,
    start_name,
    goal_name,
    start_location=None,
):
    """
    Tum rota istekleri icin tek giris noktasi.

    Gym hedefiyse ozel Gym kurali uygulanir.
    Diger hedeflerde normal A* kullanilir.

    start_location:
        main_final.py icinden vehicle.get_location() verilmesi
        tavsiye edilir.
    """

    if start_location is None:
        start_location = (
            bus_stops[
                start_name
            ].location
        )

    goal_location = (
        bus_stops[
            goal_name
        ].location
    )

    # ========================================================
    # GYM HEDEFI
    # ========================================================

    if goal_name == "gym":

        return plan_gym_route(
            planner,
            bus_stops,
            start_name,
            start_location=start_location,
        )

    # ========================================================
    # DIGER HEDEFLER -> NORMAL A*
    # ========================================================

    print(
        "\nNormal A* rotasi kullaniliyor."
    )

    print(
        f"{start_name} -> {goal_name}"
    )

    return plan_normal_route(
        planner,
        start_location,
        goal_location,
    )
