from concurrent.futures.thread import ThreadPoolExecutor

Pool = ThreadPoolExecutor(max_workers=20)

__all__ = ['Pool']
