import multiprocessing


if __name__ == "__main__":
    multiprocessing.freeze_support()
    from swphysics.gui import main

    raise SystemExit(main())
