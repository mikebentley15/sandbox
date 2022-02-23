cmake_minimum_required(VERSION 3.12)

project(socket-client-server)

add_executable(client client.c)
target_compile_options(client PRIVATE -Wall -Wextra -Werror)
install(TARGETS client RUNTIME DESTINATION bin)

add_executable(server server.c)
target_compile_options(server PRIVATE -Wall -Wextra -Werror)
install(TARGETS server RUNTIME DESTINATION bin)

add_executable(multiserver multiserver.c)
target_compile_options(multiserver PRIVATE -Wall -Wextra -Werror)
install(TARGETS multiserver RUNTIME DESTINATION bin)
