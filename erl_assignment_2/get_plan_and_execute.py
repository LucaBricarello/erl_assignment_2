#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# Messaggi e Servizi di PlanSys2
from plansys2_msgs.action import ExecutePlan
from plansys2_msgs.srv import GetPlan
from plansys2_msgs.msg import ActionExecutionInfo

class Controller(Node):

    def __init__(self):
        super().__init__('controller')
        
        # 1. Client per ottenere il piano (Simula planner_client_->getPlan)
        self.get_plan_client = self.create_client(GetPlan, '/planner/get_plan')

        # 2. Client per eseguire il piano (Simula executor_client_->start_plan_execution)
        self.execute_plan_client = ActionClient(self, ExecutePlan, '/execute_plan')

        # 3. Subscriber per il feedback (Simula action_feedback_sub_)
        self.action_feedback_sub = self.create_subscription(
            ActionExecutionInfo,
            '/action_execution_info',
            self.action_feedback_callback,
            10
        )

        self.action_completion_map = {}
        self.plan_executed = False

    def init(self):
        # Attendiamo che i servizi siano disponibili
        self.get_logger().info('Waiting for services...')
        if not self.get_plan_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Planner service not available!')
            return False
        
        if not self.execute_plan_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Executor action server not available!')
            return False
            
        self.get_logger().info('Controller Initialized.')
        return True

    def get_plan_and_execute(self):
        self.get_logger().info('Computing plan...')

        # Creiamo la richiesta. 
        # Nota: In Python non serve scaricare domain/problem text se sono già caricati nei nodi.
        # Passando stringhe vuote, il planner usa la conoscenza attuale del Domain/Problem Expert.
        req = GetPlan.Request()
        req.domain = "" 
        req.problem = ""

        # Chiamata sincrona per ottenere il piano (simile al .getPlan() del C++)
        future = self.get_plan_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().success:
            plan = future.result().plan
            print(plan)
            self.print_plan(plan)

            # Eseguiamo il piano
            self.execute_plan(plan)
        else:
            self.get_logger().error('Could not find plan to reach goal!')

    def execute_plan(self, plan):
        self.get_logger().info('Executing plan...')
        goal_msg = ExecutePlan.Goal()
        goal_msg.plan = plan
        
        # Inviamo il goal all'esecutore
        send_goal_future = self.execute_plan_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Plan rejected by executor.')
            return

        self.get_logger().info('Plan accepted, execution starting...')
        self.plan_executed = True
        
        # Richiediamo il risultato finale (opzionale, ma utile per chiudere pulito)
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info('Everything done!! (Plan Finished)')
        else:
            self.get_logger().info('Plan Failed.')
        # Chiudiamo il nodo alla fine del piano globale
        rclpy.shutdown()

    def action_feedback_callback(self, msg):
        # Questa funzione replica esattamente lo switch-case del C++
        # Il messaggio ActionExecutionInfo contiene lo status di ogni singola azione
        
        # Filtriamo messaggi vuoti o non pertinenti se necessario (come if msg->action!="0" in C++)
        if msg.action_full_name == "" or msg.action_full_name == ":0":
            return

        # Aggiorniamo la mappa di completamento
        self.action_completion_map[msg.action_full_name] = msg.completion

        # Convertiamo l'enum dello status in stringa (Switch Case logic)
        status_str = "UNKNOWN"
        if msg.status == ActionExecutionInfo.NOT_EXECUTED:
            status_str = "NOT_EXECUTED"
        elif msg.status == ActionExecutionInfo.EXECUTING:
            status_str = "EXECUTING"
        elif msg.status == ActionExecutionInfo.SUCCEEDED:
            status_str = "SUCCEEDED"
        elif msg.status == ActionExecutionInfo.FAILED:
            status_str = "FAILED"
        elif msg.status == ActionExecutionInfo.CANCELLED:
            status_str = "CANCELLED"

        # Stampiamo il log formattato come nel codice C++
        print(f"Action: {msg.action_full_name} | Completion: {msg.completion * 100.0:.1f}% | Status: {status_str}")

    def print_plan(self, plan):
        print("Plan:")
        for item in plan.items:
            print(f"  Action: {item.action}")
            print(f"  Duration: {item.duration}")
            print(f"  Time: {item.time}")

def main(args=None):
    rclpy.init(args=args)
    
    controller = Controller()
    
    if controller.init():
        controller.get_plan_and_execute()
        
        # Spin per ascoltare i feedback sul topic e attendere la fine
        try:
            rclpy.spin(controller)
        except KeyboardInterrupt:
            pass
        except Exception as e:
            # Cattura l'eccezione di shutdown generata da rclpy.shutdown()
            pass
    
    controller.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()