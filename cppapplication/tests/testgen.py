import random
numbers = [str(random.randint(0, 100)) for _ in range(100)]
numbers_str = ' '.join(numbers)
with open('in.txt', 'w') as file:
    file.write(numbers_str)