from bus_stops import list_bus_stops


# ============================================================
# DURAKLARI GÖSTER
# ============================================================

def show_bus_stops():

    stops = list_bus_stops()

    if not stops:
        raise RuntimeError(
            "Henüz durak koordinatı tanımlanmadı."
        )

    print(
        "\n=============================="
    )

    print(
        "     IKCU SHUTTLE DURAKLARI"
    )

    print(
        "=============================="
    )

    for index, stop in enumerate(
        stops,
        start=1,
    ):
        print(
            f"{index}) {stop.name}"
        )

    return stops


# ============================================================
# TEK DURAK SEÇ
# ============================================================

def choose_bus_stop(prompt):

    stops = show_bus_stops()

    print(
        "\n" + prompt
    )

    while True:

        try:

            choice = int(
                input("Seçim: ")
            )

            if 1 <= choice <= len(stops):

                selected_stop = stops[
                    choice - 1
                ]

                print(
                    "Seçilen durak:",
                    selected_stop.name,
                )

                return selected_stop

        except ValueError:
            pass

        print(
            "Geçersiz seçim. Tekrar dene."
        )


# ============================================================
# BAŞLANGIÇ + HEDEF SEÇ
# ============================================================

def choose_start_and_goal():

    print(
        "\n=== BAŞLANGIÇ DURAĞI ==="
    )

    start_stop = choose_bus_stop(
        "Başlangıç durağını seç:"
    )

    while True:

        print(
            "\n=== HEDEF DURAĞI ==="
        )

        goal_stop = choose_bus_stop(
            "Hedef durağını seç:"
        )

        if goal_stop.name != start_stop.name:
            break

        print(
            "\nBaşlangıç ve hedef "
            "aynı durak olamaz."
        )

    print(
        "\n=============================="
    )

    print(
        "Başlangıç:",
        start_stop.name,
    )

    print(
        "Hedef:",
        goal_stop.name,
    )

    print(
        "=============================="
    )

    return (
        start_stop,
        goal_stop,
    )
